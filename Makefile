# -- User definitions ---------------------------

HAL_PATH = AVR-HAL/HAL
APP_NAME = SUNS

BOARD = ARDUINOMEGA2560

INCLUDES = \
  -I . \
  -I app/inc

SRCS = \
  main.cpp


include $(HAL_PATH)/build.make