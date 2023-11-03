CC = g++
CXXFLAGS = -Wall -Os -lpthread
SRC_DIR = src
INC_DIR = include
SOURCES = $(SRC_DIR)/main.cpp $(SRC_DIR)/crc32.c
HEADERS = $(INC_DIR)/crc32.h
LIBS =
NAME = server

$(NAME): $(SOURCES) $(HEADERS)
	$(CC) $(CXXFLAGS) $(HEADERS) $(SOURCES) -o $(NAME) $(LIBS)

clean:
	rm -f $(NAME)
