Import("env")

# Set RDP level to 1 (protection enabled)
env.Append(
    UPLOAD_EXTRA_SCRIPT="""
    init
    reset halt
    stm32f1x lock 0
    reset halt
    exit
    ""