import os
import subprocess

from SCons.Script import Import
Import("env")

def make_eeprom(source, target, env):
    elf_file = os.path.join(env.subst("$BUILD_DIR"), "firmware.elf")
    eep_file = os.path.join(env.subst("$BUILD_DIR"), "firmware.eep")
    objcopy = env.subst("$OBJCOPY")
    
    print(f"--> Extracting ASM EEPROM data to: {eep_file}")
    
    cmd = [
        objcopy,
        "-j", ".eeprom",
        "--set-section-flags=.eeprom=alloc,load",
        "--change-section-lma", ".eeprom=0",
        "--no-change-warnings",
        "-O", "ihex",
        elf_file,
        eep_file
    ]
    subprocess.run(cmd)

env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", make_eeprom)