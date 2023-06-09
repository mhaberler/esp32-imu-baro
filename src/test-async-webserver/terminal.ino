/*
 * terminal - handling commands via terminal using Serial
 */


void handleTerminalCommands(){

    if(!Serial.available()) return;   // no data on terminal

    uint32_t        cnameLen        = 0;
    char            delimiters[]    = "\r\n";   // space dot comma CR NL (" .,\r\n") are possible cmd delimiters
    int             cpnt;

    cSF(command, 200);
    cSF(cname,   100);
    cSF(misc,   5000);
    cSF(misc2,    50);

    while (Serial.available()){
      misc2.clear();
      misc2.read(Serial);
      delay(10);
      misc += misc2;
    }
    misc.nextToken(command, delimiters);
    command.trim();

    // clean out any ESCAPE
    while (true) {
        if(command.length() == 0) return;
        if((int)command[0] == 27){
            Serial.printf("command starts with 27, new command: '%s', len:%i\n", command.c_str(), command.length());
            if(command.length()  < 3) return;
            command.substring(command, 3);
        }
        else break;
    }

    Serial.printf("ESP got Terminal Command: '%s'\n", command.c_str());

    cpnt = command.indexOf(' ');
    if (cpnt >= 0) { // found a space inside command
        command.substring(cname, cpnt + 1);
    }
    cname.trim();
    cnameLen = cname.length();

///////////////// begin parsing commands for actions

    if(command.startsWith("fs")){
        misc = "\n";
        if (FileSystemIsMounted()){
            addDirFileListSystem(misc, "/", 0);
            Serial.println (misc.c_str() ) ;

            printDiskUse();
        }
        else{
            misc += "Could not open File System. Is it mounted?";
            Serial.println(misc);
        }
        Serial.println();
    }

    else if(command.startsWith("remove")){
        Serial.printf("Removing file: %s\n", cname.c_str());
        removeFile(cname.c_str());
    }

    else if(command.startsWith("rename")){
        int indexspace = cname.indexOf(" ");
        cSF(fname1, 32);
        cSF(fname2, 32);
        cname.substring(fname1, 0,              indexspace + 1);
        cname.substring(fname2, indexspace + 1                );
        renameFile(fname1.c_str(), fname2.c_str());
    }

    else if(command.startsWith("print")){
        // print a filename given as 2nd parameter with option given as 3rd param
        Serial.println("Options print: <filename> hex std cam cps");

        cSF(filename, 50);
        cSF(codeword,  5);

        // cname is already trimmed
        int index = cname.indexOf(" ");                     // has cname a space? -1 if not found

        if (index >= 0) {                                   // has cname a space? Yes
            cname.substring(filename, 0, index);            // filename
            cname.substring(codeword, index, cnameLen);     // codeword
        }
        else{                                               // has cname a space? No,
            filename = cname;                               // cname is filename
            codeword = "std";                               // default
        }
        if (!filename.startsWith("/")) filename.prefix("/");

        Serial.printf("filename: '%s', codeword: '%s'\n", filename.c_str(), codeword.c_str());
        printFileContent(filename.c_str(), codeword.c_str());
    }


    else if(command.startsWith("checkfs")){
        // check file system

        checkFS();
        Serial.printf("FSCorruptState: %i:  %s\n", FSCorruptState, FSCorruptType[FSCorruptState]);
    }

    else if(command.startsWith("check")){
        // check DATACAM file for readability

        checkFile(DATACAM);
    }


    else if(command.startsWith("make")){
        // make DATACAM file in desired size

        Serial.println("Options make: <size in units of 1k (1024) Bytes>");

        if (cname.length() > 0){
            long fsize;
            cname.toLong(fsize);          // convert string cname to Long fsize
            makeCAMFile(fsize * kibi);
        }
    }

    else if(command.startsWith("format")){
        Serial.println("Options format: FLASH");

        if (cname.startsWith("FLASH")){
            formatFileSystem();
        }
        else{
            Serial.println("NOT formatting Flash - you must type exactly: 'format FLASH'");
        }
    }

    else{
        Serial.println("Invalid command");
    }
}
