//! Helpers shared between bootloader and app for identity persistence and
//! protocol housekeeping. These all take a `&CommonApi` and are usable
//! verbatim from either image.
//!
//! Kept in `shared` so neither the BL nor the app needs to ship its own
//! copy — and so that future changes touch one place.

use crate::common_api::CommonApi;
use crate::flash_layout::{USER_STORE_BASE, USER_STORE_SIZE};
use crate::params_store::{self, Record, ScanResult, SLOT_LEN};
use crate::protocol::{Command, Message, ProtocolData};
use crate::user_store::{self, Slot};

/// Snapshot the entire `USER_STORE` flash page as a slice. Safe because
/// the region is mapped read-only flash.
fn page_bytes() -> &'static [u8] {
    // SAFETY: USER_STORE region is mapped read-only flash; reading is safe.
    unsafe {
        core::slice::from_raw_parts(USER_STORE_BASE as *const u8, USER_STORE_SIZE as usize)
    }
}

/// Read the user_store record from flash. Falls back to factory defaults
/// (serial 0xCD00_0001, unassigned id) if the page is blank or fails CRC.
pub fn load_identity(_api: &CommonApi) -> Slot {
    user_store::parse(page_bytes()).unwrap_or_else(Slot::factory_default)
}

/// Read the latest valid motor-parameter record from flash, if any.
/// Returns `None` if the page is blank, every slot is corrupt, or no
/// record has ever been written.
pub fn load_params(_api: &CommonApi) -> Option<Record> {
    params_store::scan(page_bytes()).latest
}

/// Persist an updated identity record. Bumps `seq`, recomputes CRC,
/// erases the user_store page, programs the leading record, and
/// re-emits any current motor-parameter record into slot 0 so changing
/// the id does not wipe previously-saved tunes.
/// Returns `true` on success.
pub fn persist_identity(api: &CommonApi, identity: &mut Slot) -> bool {
    // Capture latest params before we erase the page.
    let preserved = params_store::scan(page_bytes()).latest;

    identity.seq = identity.seq.wrapping_add(1);
    let id_bytes = identity.finalize();
    if (api.flash_unlock)() != 0 { return false; }
    if (api.flash_erase_page)(USER_STORE_BASE) != 0 {
        (api.flash_lock)();
        return false;
    }
    if (api.flash_program)(USER_STORE_BASE, id_bytes.as_ptr(), id_bytes.len()) != 0 {
        (api.flash_lock)();
        return false;
    }
    if let Some(r) = preserved {
        // Restart the params log at slot 0 with the latest values.
        let bytes = r.to_bytes();
        let _ = (api.flash_program)(params_store::slot_addr(0), bytes.as_ptr(), bytes.len());
    }
    (api.flash_lock)();
    true
}

/// Persist a new motor-parameter record. Tries to append into the next
/// erased slot to minimise erase cycles; only re-erases the whole page
/// (preserving the identity record + the current params) when all
/// seven slots have been programmed.
/// Returns `true` on success.
pub fn persist_params(api: &CommonApi, values: &[f32]) -> bool {
    let scan = params_store::scan(page_bytes());
    let next = Record::from_values(scan.next_seq(), values);
    let bytes = next.to_bytes();

    if (api.flash_unlock)() != 0 { return false; }
    let ok = match scan.first_free {
        Some(slot) => {
            // Fast path: append into a still-erased slot, no page erase.
            (api.flash_program)(params_store::slot_addr(slot), bytes.as_ptr(), bytes.len()) == 0
        }
        None => persist_params_full_rewrite(api, &bytes),
    };
    (api.flash_lock)();
    ok
}

/// Slow path: every parameter slot is programmed. Read back the current
/// identity record (so we don't lose it), erase the page, then re-emit
/// identity at base + new params at slot 0. Caller has already
/// `flash_unlock`-ed and is responsible for `flash_lock`-ing on return.
fn persist_params_full_rewrite(api: &CommonApi, new_bytes: &[u8; SLOT_LEN]) -> bool {
    // Snapshot identity before we wipe the page.
    let id = user_store::parse(page_bytes()).unwrap_or_else(Slot::factory_default);
    if (api.flash_erase_page)(USER_STORE_BASE) != 0 { return false; }
    let id_bytes = id.finalize();
    if (api.flash_program)(USER_STORE_BASE, id_bytes.as_ptr(), id_bytes.len()) != 0 {
        return false;
    }
    (api.flash_program)(params_store::slot_addr(0), new_bytes.as_ptr(), new_bytes.len()) == 0
}

/// Returns scan results for diagnostic / test code that wants to know
/// how full the params page is.
pub fn params_scan() -> ScanResult {
    params_store::scan(page_bytes())
}

/// Build a device-side `Control` reply (controller bit clear).
#[inline]
pub fn build_reply(device_id: u8, cmd: Command, data: ProtocolData) -> Message {
    Message::Control { device_id, is_controller: false, cmd, data }
}

/// Send a `DiscoveryReq` carrying our serial. `previous_id` is 0 when
/// we have no assignment yet, otherwise the id from user_store.
pub fn send_discovery_req(api: &CommonApi, identity: &Slot) {
    let m = Message::DiscoveryReq {
        serial: identity.serial_no,
        previous_id: if identity.assigned_id == Slot::UNASSIGNED_ID {
            0
        } else {
            identity.assigned_id
        },
    };
    if let Ok(out) = m.encode() {
        let _ = (api.can_send)(&out as *const _);
    }
}

/// Send a Message via CAN, spinning on the TX mailbox up to `max_spins`
/// loop iterations. Returns true if it landed in a mailbox.
pub fn send_blocking(api: &CommonApi, msg: &Message, max_spins: u32) -> bool {
    let out = match msg.encode() {
        Ok(f) => f,
        Err(_) => return false,
    };
    let mut spins = 0u32;
    while (api.can_send)(&out as *const _) == 0 {
        if spins >= max_spins { return false; }
        core::hint::spin_loop();
        spins += 1;
    }
    true
}
