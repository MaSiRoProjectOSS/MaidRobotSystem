@echo off
FOR /F "usebackq delims== tokens=1,2" %%a IN ("src/vosk.config.ini") DO SET %%a=%%b
rem source src/vosk.config.ini
rem ##------------------------------------------
python -m pip install --upgrade pip
pip install --upgrade websockets
pip install --upgrade sounddevice
rem ##------------------------------------------
docker build -t vosk:%VOSK_MODEL% -f src/Dockerfile --build-arg MODEL_NAME=%VOSK_MODEL% .
docker stop vosk_server_%VOSK_MODEL%
docker container rm vosk_server_%VOSK_MODEL%
docker container run --name vosk_server_%VOSK_MODEL% -d -p %SERVER_PORT%:2700 vosk:%VOSK_MODEL%
rem ##------------------------------------------

