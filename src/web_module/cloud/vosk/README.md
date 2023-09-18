# README

* This project to get the speech recognition VOSK working.
* Please Install "Docker" and "Python".
* The shell assumes a Windows command prompt.
* The model is controlled by ``vosk/src/vosk.config.ini``.

## Usage

1. Edit the ``vosk/src/vosk.config.ini``.
   1. Copy and paste one of the following into ``vosk/src/vosk.config.ini``.
      * ``vosk/src/vosk-model/vosk-model-en-us-0.42-gigaspeech/vosk.config.ini``
        * **gigaspeech** downloads 2.3 GB in size.
      * ``vosk/src/vosk-model/vosk-model-small-en-us-0.15/vosk.config.ini``
1. Install Docker image and run container.
   1. Run ``00_make_image.cmd``.
      * This refers to ``vosk/src/vosk.config.ini``.
      * Already existing containers will be removed.
1. Run the Vosk
   1. Execute ``01_run.cmd``. so Let's speak.


## Add a model

1. Edit the vosk.config.ini.
   1. Write the model name in the VOSK_MODEL of the vosk.config.ini
   1. Specify any number (e.g. 2700) for SERVER_PORT.
      * Make sure that the port does not cover other models. Otherwise, container creation will fail.

For details on models, see the following [https://alphacephei.com/vosk/models](https://alphacephei.com/vosk/models)

