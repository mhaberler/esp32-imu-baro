/*
 * File System functions - works for all file systems: FFat, SPIFFS, LittleFS
 */


void initFileSystem() {

//FFat
    #if myFSCode == 1
        //~Serial.printf("using FFat\n");

        // from FFat.h: bool begin( bool            formatOnFail   = false,
        //                          const   char *  basePath       = "/ffat",
        //                          uint8_t         maxOpenFiles   = 10,
        //                          const   char *  partitionLabel = (char*)FFAT_PARTITION_LABEL) (="ffat")
        //                        );
        bool            formatOnFail = false;
        const char *    basePath     = "/ffat";
        uint8_t         maxOpenFiles = 10;
        bool            result       = myFS->begin(formatOnFail, basePath, maxOpenFiles);
        //~Serial.printf("begin FFat done\n");

//SPIFFS
    #elif myFSCode == 2
        //~Serial.printf("using SPIFFS \n ");

        // from SPIFFS.h: bool begin( bool            formatOnFail   = false,
        //                            const   char *  basePath       = "/spiffs",
        //                            uint8_t         maxOpenFiles   = 10,
        //                          );
        bool            formatOnFail = false;
        const char *    basePath     = "/spiffs";
        uint8_t         maxOpenFiles = 10;
        bool            result       = myFS->begin(formatOnFail, basePath, maxOpenFiles);
        //~Serial.printf("begin SPIFFS done\n");


//LittleFS
    #elif myFSCode == 3
        //~Serial.printf("using LittleFS\n");

        // from LITTLEFS.h: bool begin( bool         formatOnFail = false,
        //                              const char * basePath     = "/littlefs",
        //                              uint8_t      maxOpenFiles = 5
        //                             );
        bool            formatOnFail = false;
        const char *    basePath     = "/littlefs";
        uint8_t         maxOpenFiles = 10;
        bool            result       = myFS->begin(formatOnFail, basePath, maxOpenFiles);

    #endif

    cSF(iFS, 500);
    iFS.printf("Initialising File System '" myFSName "'");

    if (result){
    // success
        bFSIsMounted = true;
        iFS.printf(" -- successful! %i Bytes free, FillLevel:%0.1f%%", getFreeBytes(), getFillLevel());
        Serial.println(iFS);
        iFS.clear();
    }
    else {
    // failure
        bFSIsMounted = false;
        iFS += " -- FAILURE!";
        Serial.println(iFS);
    }
}


void endFileSystem() {

    myFS->end();
}


bool FileSystemIsMounted(){
// gibt es keinen "echten" mount test?
// myFS->exists("/") ergibt false! exists kann nicht auf rootdir testen !?

    bool flag;

    File test = myFS->open("/");
    if (!test) flag = false;
    else       flag = true;

    test.close();
    return flag;
}


void createDirectory(SafeString & misc, const char * path){
// create a dir

    misc.clear().printf("Creating Dir:  %-10s --- ", path);
    if (myFS->exists(path))    misc += "Exists";
    else {
        if(myFS->mkdir(path))  misc += "Directory created";
        else                   misc += "FAILURE creating directory";
    }
}


uint32_t getTotalBytes(){
    return myFS->totalBytes();
}


uint32_t getUsedBytes(){

    uint32_t used;

    #if   myFSCode == 3 // LittleFS
        used = myFS->usedBytes();

    #elif myFSCode == 2 // SPIFFS
        used = myFS->usedBytes();

    #elif myFSCode == 1 // FFat
        used = getTotalBytes() - getFreeBytes();

    #endif

    return used;
}


uint32_t getFreeBytes(){
// problem: while all FS have totalBytes,
// SD + SPIFFS have usedBytes but no freebytes, while
// FFat has only freeBytes, but no usedbytes

// FFat :
//    size_t totalBytes();
//    size_t freeBytes();

// SPIFFS:
//    size_t totalBytes();
//    size_t usedBytes();

// LittleFS
//    size_t totalBytes();
//    size_t usedBytes();
//
// workaraound: add into LITTLEFS.h:
//                  size_t freeBytes();
//              and into LITTLEFS.cpp:
//                  size_t LITTLEFSFS::freeBytes(){return totalBytes() - usedBytes();}

    uint32_t free;

    #if   myFSCode == 3   // LittleFS
        free = getTotalBytes() - getUsedBytes();

    #elif myFSCode == 2 // SPIFFS
        free = getTotalBytes() - getUsedBytes();

    #elif myFSCode == 1 // FFat
        free = myFS->freeBytes();

    #endif

    return free;
}


float getFillLevel(){

    float    fsFillLevel = NAN;                    // means filllevel is invalid
    uint32_t totalbytes  = getTotalBytes();
    if (totalbytes > 0) fsFillLevel = 100.0f * getUsedBytes() / totalbytes;     // in percent

    return fsFillLevel;
}


void printDiskUse(){

    uint32_t total = getTotalBytes();
    uint32_t used  = getUsedBytes();
    uint32_t free  = getFreeBytes();

    Serial.printf("Disk usage:\n");
    Serial.printf("              Bytes     Bytes Blocks      %%Ttl\n");
    Serial.printf("    Total: %8u  0x%06X %6.1f  %7.3f%%\n", total, total, total / 4096.0f, 100.0f);
    Serial.printf("    Used:  %8u  0x%06X %6.1f  %7.3f%%\n", used,  used,  used  / 4096.0f, 100.0f * used / total );
    Serial.printf("    Free:  %8u  0x%06X %6.1f  %7.3f%%\n", free,  free,  free  / 4096.0f, 100.0f * free / total );
}


// test for file existence
bool existsFile(const char * filename){

    if (myFS->exists(filename)) return true;
    else                        return false;
}


// call like :  addDirFileListSystem(misc, "/", 0)
void addDirFileListSystem(SafeString & flist, const char * dirname, uint8_t level){
// Note: 'file.size()' of a dir is always == 0 (FFat, LittleFS)

            auto                t0 = micros();

            File                root;
            File                file;
    static  uint32_t            totalfsize;
    static  uint32_t            totalBfsize;
            uint32_t            totalB = myFS->totalBytes();
    const   unsigned char       bad[]  = {'/', 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, '.', 0xFF, 0xFF, 0xFF, 0x00};  // 14 chars

                                FSCorruptState = 0;

    if (level == 0) {
        if (! FileSystemIsMounted()){
            flist += "Cannot open File System. Is it mounted?";
            return;
        }

        totalfsize  = 0;
        totalBfsize = 0;
        flist      +=  ("Type of Size:             FileSize      FlashSize \n");        // resets flist
        flist      +=  ("DIR FILE                Bytes  %Ttl   Bytes Blocks\n");
    }

    root = myFS->open(dirname);
    if(!root){
        FSCorruptState = 2; // Failed to open directory '/'
        flist.printf("ERR: Failed to open directory '%s'", dirname);
        return;
    }

    if(!root.isDirectory()) {
        FSCorruptState = 3; // '/' not seen as directory
        flist.printf("ERR: '%s' is not a directory",       dirname);
        return;
    }

    flist.printf("%-49s\n", root.name());

    file = root.openNextFile();
    while(file){
        if (strcmp(file.name(), (const char *)bad) == 0){
            FSCorruptState = 1;                     // found corrupt file
            break;
        }
        else if(file.isDirectory()){
            addDirFileListSystem(flist, file.name(), level + 1);
            flist += "\n";
        }
        else {
            size_t fsize  = file.size();                    // the size of the file
            size_t Bfsize;
            if (fsize % 4096 == 0)  Bfsize = fsize;
            else                    Bfsize = (fsize / 4096 + 1) * 4096;  // the space the file occupies on the FS; min space = 1block of 4096 bytes

            float  fperc  = 100.0 * fsize  / totalB;
            flist.printf("    %-17s%8i %4.1f%% %7i  %5i\n", file.name(), fsize, fperc, Bfsize, Bfsize / 4096);
            totalfsize  += fsize;
            totalBfsize += Bfsize;
        }
        file = root.openNextFile();
    }

    if (FSCorruptState == 0){
        if (level == 0) {
            flist.printf("TOTAL:  %13s%8i %4.1f%% %7i  %5i\n", "", totalfsize, (float)totalfsize * 100.0 / (float)totalB, totalBfsize, totalBfsize / 4096 );
        }
    }
    else{
        flist.printf("FS is corrupted, type: %s\n", FSCorruptType[FSCorruptState]);
    }

    // float dur = (micros() - t0) / 1000.0f;
    // flist.printf("overall: %0.1f ms\n", dur);
}


void checkFS(){
// checks for corrupted file system
// meaningful only on FFat

                   auto  t0 = micros();
                   File  file;
                   File  root;
    const unsigned char  bad[] = {'/', 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, '.', 0xFF, 0xFF, 0xFF, 0x00};

    FSCorruptState = 0;

    root = myFS->open("/");
    if     (!root)                FSCorruptState = 2; // Failed to open directory '/'
    else if(!root.isDirectory())  FSCorruptState = 3; // '/' not seen as directory
    else {
        // walk through all files of root dir; break on corrupted file
        file = root.openNextFile();
        while (file) {
            // Serial.printf("file.name(): %12s file.size(): %i   ", file.name(), file.size()); //
            // Serial.printf("this file exists %i  ", myFS->exists(file.name())); //
            // Serial.printf("  %0.1f ms\n", (micros() - t0) / 1000.0f);

            if (strcmp(file.name(), (const char *)bad) == 0){
                FSCorruptState = 1;                     // found corrupt file; seems to work only on FFat
                break;
            }
            file = root.openNextFile();
        }
    }

    // float dur = (micros() - t0) / 1000.0f;
    // Serial.printf("checkFS: overall %0.1f ms\n", dur);
}


void checkFile(const char * filename){
    // simply try to read all bytes in a file

    File  file;
    bool  ok = true;
    cSF(misc, 100);
    misc.printf("checkFile: Filename %s  ", filename);

    if (myFS->exists(filename)){
        if (!myFS->open(filename)){
            Serial.printf("%s cannot be opened\n", misc.c_str() );
        }
        else{
            uint32_t counter = 0;
            while(file.available()){
                uint8_t fr = file.read();
                if (fr < 48 or fr > 86) ok = false;
                counter++;
            }

            misc += "read bytes ";
            if (ok and counter == file.size())  misc += "--> All OK!";
            else                                misc += "--> NOT OK!";
            Serial.println(misc);
        }
    }
    else{
        Serial.printf("%s does not exist\n", misc.c_str());
    }
}


// if no data file exists then make standard file in loop
void initDataFile(){

    Serial.println("Initializing Data File");
    const char *    fn = DATACAM;
    const uint32_t  fs = getFileSize(fn);
    if (fs == 0){
      dataFilekB = defaultdataFilekB; // data file will be made in Loop
      Serial.printf("   Preparing to make Data File: '%s', size: %0.1f kB %0.0f Bytes, \n", fn, dataFilekB, dataFilekB * kibi);
    }
    else{
      Serial.printf("   Found Data File: '%s', size: %0.1f kB, %u Bytes\n", fn, (fs / kibi), fs);
    }
}


void makeCAMFile(uint32_t fsize){
    // fsize is filesize in bytes
    // file is written with 32 printable ASCII chars per record
    // with 4MB ESP32 max is 1460 kB -> 1495040 Bytes

    const   uint8_t     bpr         = 32;     // size of a record
    const   uint8_t     record[bpr] = {48,49,50,51,52,53,54,55,56,57,                                       // coding: 0 ... 9
                                       65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86};  // coding: A ... V; no terminating \0 !
            File        file;

    auto t0 = micros();
    makingDataFile = true;
    Serial.printf("makeCAMFile: with size: %i\n", fsize);

    file = myFS->open(DATACAM, "w");     // clear file
    file.close();

    file = myFS->open(DATACAM, "rb+");   // open for binary reading and writing
    for(uint32_t i = 0; i < fsize / bpr; i++){
        file.write(record, bpr);
    }
    file.close();

    float dur = (micros() - t0) / 1000.0f;
    Serial.printf("makeCAMFile: finished in %0.1f ms\n", dur);
    makingDataFile = false;
}


void renameFile(const char * oldname, const char *  newname){
// rename a file from oldname to newname

    Serial.printf("Renaming file %s to %s \n", oldname, newname);
    // Serial.printf("File %s exists: %i\n", oldname, myFS->exists(oldname));

    if (myFS->rename(oldname, newname)) Serial.println("File renamed");
    else                                Serial.println("Rename failed");
}


void removeFile(const char * filename){
// bool VFSImpl::exists(const char* path)
// on SPIFFS remove fails if filename does not exist; check with exists first

    cSF(rf, 200, "REMOVE FILE ");

    if (myFS->exists(filename)){
        if (!myFS->remove(filename)) rf.printf("'%s' Failed with error", filename);
        else                         rf.printf("'%s' Done OK", filename);
    }
    else                             rf.printf("'%s' - not possible, it does not exist", filename);

    Serial.println(rf);
}


// call like: printFileContent(filename, "hex"); or "std" 
void printFileContent(const char * filename, const char * printtype ){

    if (!myFS->exists(filename)) {
        Serial.printf("printFileContent: File '%s' does not exist\n", filename);
        return;
    }

    Serial.printf("printFileContent: file: '%s' as type '%s':\n", filename, printtype);

    File file = myFS->open(filename, "r");
    if(!file){
        Serial.printf("   Failed to open file '%s' for reading\n", filename);
        return;
    }

    uint32_t byteCounts = 0;

    while(file.available()){

        if (strstr(printtype, "hex")) {             // printtype == "hex"
            uint8_t fr = file.read();
            byteCounts++;
            Serial.printf("%02X ", fr);
            if (byteCounts % 64 == 0) Serial.printf("\n");
        }

        else{                                       // printtype == "std" or ""
            uint8_t fr = file.read();
            byteCounts++;
            Serial.write(fr);
        }
    }

    Serial.printf("Bytes:%i  Size:%i\n\n", byteCounts, file.size());
    file.close();
}


bool formatFileSystem(){

    bool formatted = false;
    cSF(msg, 100);

    Serial.printf("Formatting File System '%s'   ", myFSName);

//FFat
    //bool format(bool full_wipe = FFAT_WIPE_QUICK, char* partitionLabel = (char*)FFAT_PARTITION_LABEL);
    //Wipe disk: 'quick' just wipes the FAT. 'full' zeroes the whole disk
    //              #define FFAT_WIPE_QUICK 0
    //              #define FFAT_WIPE_FULL 1
    #if   myFSCode == 1
        formatted = myFS->format(FFAT_WIPE_FULL);

//SPIFFS
    //bool format();
    #elif myFSCode == 2
        formatted = myFS->format();

//LittleFS
    //bool format();
    #elif myFSCode == 3
        formatted = myFS->format();

    #endif

    if(formatted) msg = "Success";
    else          msg = "FAILURE";

    Serial.println(msg);

    return formatted;
}


uint32_t getFileSize(const char * filename){
// get size in bytes of any file
// returns 0 for non-existing file

    uint32_t fsize;

    if (myFS->exists(filename)){
        File file  = myFS->open(filename, "r");
        fsize = file.size();
        file.close();
    }
    else fsize = 0;

    // Serial.printf("getFileSize: %s, %i, %0.1f kB\n", filename, fsize, fsize / kibi);
    return fsize;
}
