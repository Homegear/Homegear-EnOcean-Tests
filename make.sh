#!/bin/bash
g++ -std=c++11 -o homegear-enocean-tests $1 main.cpp -lhomegear-base -lgcrypt -lpthread -lgnutls
g++ -std=c++11 -o sniff $1 sniff.cpp -lhomegear-base -lgcrypt -lpthread -lgnutls
