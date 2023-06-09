/*
 * Async Web server code
 */


AsyncWebServer      server(80);                // instance of the async webserver

#define CHUNK  0
#define STATIC 1

static void downloadPage(AsyncWebServerRequest* request){

    Serial.println("2 webserver: downloadPage");
    deferredRequest = request;
}


void initWebServer() {

    Serial.println("Initializing Web Server");

// root page
    server.on("/",              HTTP_GET, [] (AsyncWebServerRequest *request) {request->send(200, "text/html",   getPagecodeData (request));});

// handle the make data file
    server.on("/makeDataFile",  HTTP_GET, [] (AsyncWebServerRequest *request) {handleMakeDataFile(request);});

// Data HUMAN READABLE
    // download the data file in deferred mode and present in human readable format
    server.on("/htmldataST",    HTTP_GET, [] (AsyncWebServerRequest *request) {
                                                                                cSF(pagecode, 2000);
                                                                                getPagecodeBinData(pagecode, STATIC);
                                                                                request->send(200, "text/html",  pagecode.c_str()  );
                                                                               });

// Data BINARY
    // downloading the data file in STATIC mode as BINARY files
    // server.serveStatic("/bindataST",      *myFS, "/data.cam");

    // downloading the data file in deferred mode as BINARY files
    server.on("/bindataST", HTTP_GET, downloadPage);


// Finish Init
    server.begin(); // has void return
}


void handleMakeDataFile(AsyncWebServerRequest *request){
// does nothing except setting dataFilekB to web input value;
// file is then made from Loop   

    Serial.print("handleMakeDataFile: ");

    if(request->hasParam("filesize")){
        AsyncWebParameter* p = request->getParam("filesize");
        dataFilekB = p->value().toFloat();
    }
    else{
        dataFilekB = defaultdataFilekB;
    }

    Serial.printf("%0.0f Bytes  %0.1f kB\n", dataFilekB * kibi, dataFilekB );

    request->redirect("/");
}


String getPagecodeData(AsyncWebServerRequest *request){

    Serial.println("getPagecodeData: ");

    cSF(html_data, 5000);

html_data = (R"=====(
<!DOCTYPE html>
<html lang='en'>
<meta name='viewport' content='width=device-width, initial-scale=1.0, user-scalable=yes'>
<meta charset='utf-8' />
<style>
html           {font-family:Helvetica; display:inline-block; margin:0px auto; text-align:center; font-size:20px;}
body           {margin:5px;}
h1, h2, h3     {color:#444444; font-weight:bold;}
h1             {margin:10px auto  0px; font-size:40px;}
h2             {margin:10px auto  0px; font-size:30px;}
h3             {margin: 0px auto 10px; font-size:24px;}
h2.info        {font-weight:normal; max-width:640px; font-size:28px; text-align:left;}
h2.status      {font-weight:normal; font-size:28px;}
input          {font-size:20px;}
b              {font-weight:900;}
.bold          {font-weight:900;}

</style>
)=====");

    html_data +=    "<title>TEST_async_server</title>"
                    "<h2>Data</h2>\n"
                    "<div >\n"
                        "<span style='color:red; font-weight:bold'>Watch the Serial Terminal output for response messages!</span>"
                        "<p class='bold'>Show Data in Human-Readable Format\n"
                        "<br><p class='ptop'>"
                            "<a style='font-size:28px;' href='/htmldataST' target='_blank'>Decoded Data</a>"

                        "<br><br>"

                        "<p class='bold'><br>Download Data in Binary Format\n"
                        "<br><p class='ptop'>"
                            "<a style='font-size:28px;' href='/bindataST'  target='_blank'>Binary Data</a>"
                        "<br><br>";

    // File Listing
    html_data +=        "<h2>Dirs & Files</h2>\n"
                            "<pre>";
                                if (makingDataFile) html_data += "<span style='color:red;'>Making data-file has not finished;\nrefresh page to update</span>";
                                else                addDirFileListSystem(html_data, "/", 0);
    html_data +=            "</pre>\n";

    html_data.printf(   "<p class='bold'>To change the size of the data file\n"
                        "<br>enter the new file size in <b><span class='A'>kB</span></b>"
                        "<form action='/makeDataFile' method='get' >\n"
                            "<input type='text' name='filesize' size='6' value='%0.1f'>\n"
                            "<input class='submit' type='submit' value='Submit'>"
                        "</form>\n"
                            , defaultdataFilekB);

    html_data +=    "</div>";

    return html_data.c_str();
}


/*
 * JS coding for getting binary data and printing in human readable format
 */
const char * JScodeBinData = R"=====(
async function handleBinData(){
    let BinDataSource = "%s";
    console.log("handleBinData start, datasource: ", BinDataSource);

    let t0              = performance.now();
    const fetchdata     = await fetch(BinDataSource, {cache: "no-store"});
    const fetchbuffer   = await fetchdata.arrayBuffer();
    let t1              = performance.now();
    let dt              = t1 - t0;
    let fetchBytes      = fetchbuffer.byteLength;
    console.log("fetchbuffer: ", fetchBytes, "B ", dt.toFixed(1), "ms ", (fetchBytes/dt).toFixed(1), "kB/s" );
    console.log(fetchbuffer);

    let utf8decoder = new TextDecoder(); // default 'utf-8' or 'utf8'
    let counter = 1;
    let reclen  = 32;
    let ddata   = "";
    for (let i = 0; i < fetchBytes; i += reclen){
        ddata += counter++ + " " + utf8decoder.decode(fetchbuffer.slice(i, i + reclen)) + "\n"
    }
    let t2              = performance.now();
    let dtjs            = t2 - t1;

    let strdata = "<pre>";
    strdata += "#Binary Data in Human-Readable Format - %s\n";
    strdata += "#Download: " + fetchBytes/1024 +  " kB, " + fetchBytes +  " bytes, " + fetchBytes/reclen + " records, dur: " + dt.toFixed(0) + " ms, speed: " + (fetchBytes/dt).toFixed(0) + " kB/s\n";
    strdata += "#PageCalc: " + dtjs.toFixed(0) + " ms\n"
    strdata += "#Overall:  " + (t2 - t0).toFixed(0) + " ms\n";
    strdata += ddata;

    document.body.innerHTML = strdata;
}
)=====";


void getPagecodeBinData(SafeString & pagecode, uint8_t servetype){

    Serial.printf("1 webserver: getPagecodeBinData: servetype: %s\n", (servetype == CHUNK) ? "Chunk" : "Static");

    const char * bcode;
    const char * hcode;

    bcode = "/bindataST";
    hcode = "/htmldataST";

    // select refreshcode setting for refreshing page every 1 ... 30 sec
    cSF(refreshcode, 100);
    // @12MB file, download takes ~18sec sec
    // @200kB file, download takes <0.4 sec
    // refreshcode.printf("<meta http-equiv='refresh' content='30.0; url=%s'>\n", hcode);
    // refreshcode.printf("<meta http-equiv='refresh' content='10.0; url=%s'>\n", hcode);
    refreshcode.printf("<meta http-equiv='refresh' content='5.0; url=%s'>\n", hcode);
    // refreshcode.printf("<meta http-equiv='refresh' content='3.0; url=%s'>\n", hcode);
    // refreshcode.printf("<meta http-equiv='refresh' content='1.0; url=%s'>\n", hcode);

    pagecode  =     "<!DOCTYPE html>\n"
                    "<html lang='en'>\n"
                    "<meta name='viewport' content='width=device-width, initial-scale=1.0, user-scalable=yes'>\n"
                    "<meta charset='utf-8' />\n";

    // next line sets auto-refresh of web page; comment out to avoid page refreshing
    pagecode +=     refreshcode.c_str();

    pagecode +=     "<style> html {white-space:nowrap;} </style>\n"
                    "<title>CAM Data</title>\n"
                    "<pre>Waiting for data ...</pre>\n"
                    "<script>\n";

    pagecode.printf(JScodeBinData, bcode, bcode);  // insert JS function handleBinData()

    pagecode +=     "handleBinData();\n"
                    "</script>\n";
}
