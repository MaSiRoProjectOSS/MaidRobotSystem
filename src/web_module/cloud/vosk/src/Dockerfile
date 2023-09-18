FROM alphacep/kaldi-vosk-server:latest

ARG MODEL_NAME=vosk-model-en-us-0.42-gigaspeech

ENV MODEL_VERSION 0.22
RUN mkdir /opt/vosk-model \
   && cd /opt/vosk-model \
   && wget -q https://alphacephei.com/vosk/models/${MODEL_NAME}.zip \
   && unzip ${MODEL_NAME}.zip \
   && rm -rf ${MODEL_NAME}.zip
RUN cd /opt/vosk-model \
   && mv ${MODEL_NAME} model

EXPOSE 2700
WORKDIR /opt/vosk-server/websocket
CMD [ "python3", "./asr_server.py", "/opt/vosk-model/model" ]
