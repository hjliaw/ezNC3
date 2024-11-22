import os, shutil
Import("env")

penv = env["PIOENV"]             # ezNC2, Tinybee, ...
pbdr = env["PROJECT_BUILD_DIR"]  # /home/.../.pio/build

fpary = [ "libdeps", penv, "ESP8266 and ESP32 OLED driver for SSD1306 displays", "src", "mfont.h"]
mfont = os.path.join( os.path.dirname(pbdr), *fpary )
fpath = os.path.dirname( mfont )

cfonts = os.path.join( env["PROJECT_DIR"], "fonts" )

print("INFO: file:", mfont)   
if os.path.isfile( mfont ):
    print( "\t", fpary[-1], "file exists")
else:
    print( "\tcopy custom fonts for env", penv)
    for item in os.listdir(cfonts):
        src_path = os.path.join(cfonts, item)
        dest_path = os.path.join(fpath, item)
        if os.path.isfile(src_path):  # no sub-dir
            shutil.copy2(src_path, dest_path)
       
# from chatgpt, not working    
def before_build(env, platform):
    print("Running pre-compilation tasks...")

    environment_name = env.subst("$PIOENV")
    build_dir = env.subst("$BUILD_DIR")
    
    print(f"environment: {environment_name}")
    print(f"  build dir: {build_dir}")

    with open(f"{build_dir}/{environment_name}_build_info.txt", "w") as file:
        file.write(f"Environment: {environment_name}\n")
        file.write("Custom build info generated before compilation.\n")

    
