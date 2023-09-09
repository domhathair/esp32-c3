CC = gcc
CXXFLAGS = -Wall -Os -lGL -lm -lpthread -ldl -lrt -lX11
SRC_DIR = src
INC_DIR = include
GUI_DIR = raygui/src
SOURCES = $(SRC_DIR)/main.c
HEADERS = $(GUI_DIR)/raygui.h
LIBS =
NAME = server

$(NAME): $(SOURCES) $(HEADERS)
	$(CC) $(CXXFLAGS) $(HEADERS) $(SOURCES) -o $(NAME) $(LIBS)

clean:
	rm -f $(NAME)
