import os
import json
import logging
import shutil
import argparse

ROOT_PATH = os.path.dirname(__file__)
ELF_PATH = ROOT_PATH + "/Loader"
# Get STM32CubeProgrammer
STM32_PRG_PATH = os.environ.get("STM32_PRG_PATH")
if STM32_PRG_PATH is None:
    raise(ImportError("Please install STM32CubeProgrammer"))

def logger_init() -> logging.Logger:
    # Create and configure logger
    logging.basicConfig(
        level=logging.INFO,  # Set the logging level (e.g., DEBUG, INFO, WARNING, ERROR, CRITICAL)
        format='%(levelname)s - %(message)s',  # Format for log messages
        handlers=[
            # logging.FileHandler("app.log"),  # Log to a file
            logging.StreamHandler()  # Log to the console
        ]
)
    # Create a logger object
    return logging.getLogger("logger")

if __name__ == "__main__":
    logg = logger_init()
    
    parser = argparse.ArgumentParser(description="Get elf name")
    parser.add_argument("elf_name", help="ELF name")
    args = parser.parse_args()
    ELF_NAME = str(args.elf_name).split(".")[0]
    logg.info(f"Project name: {ELF_NAME}")
    
    # Get all needed paths used through the program
    STM32_PRG_LOADER_PATH = os.path.join(STM32_PRG_PATH, "ExternalLoader")
    STM32_PRG_LOADER_STLDR_FILE = os.path.join(STM32_PRG_LOADER_PATH, ELF_NAME + ".stldr")
    ELF_FILE = os.path.join(ELF_PATH, ELF_NAME + ".elf")
    STLDR_FILE = os.path.join(ELF_PATH, ELF_NAME + ".stldr")
        
    # Generate stdlr extension file (it is just the elf file)
    # Copy external loader to the STM32CubeProgrammer loaders directory
    try:
        shutil.copyfile(src=ELF_FILE, dst=STLDR_FILE)
        shutil.copyfile(src=STLDR_FILE, dst=STM32_PRG_LOADER_STLDR_FILE)
    except Exception as e:
        logg.error(f"{e} Copying file failed")

    logg.info(f"External loader copied to {STM32_PRG_LOADER_STLDR_FILE}")