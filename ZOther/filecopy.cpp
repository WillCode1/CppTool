//实现单链表中的各种操作

#include "pch.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>


int main() {
	FILE *fp;

	{
		if (fopen_s(&fp, "./abc.txt", "wb"))
		{
			printf("Cannot open the file!\n");
			exit(0);
		}

		const auto write = "Hello World!";
		for (int i = 0; i < 100; i++)
		{
			fwrite(write, sizeof(char), strlen(write), fp);
		}
		fclose(fp);
	}

	{
		if (fopen_s(&fp, "./abc.txt", "rb"))
		{
			printf("Cannot open the file!\n");
			exit(0);
		}

		int size;
		char buffer[100] = { 0 };
		while ((size = fread(buffer, sizeof(char), sizeof(buffer) - 1, fp) > 0))
		{
			std::cout << buffer << std::endl;
			memset(buffer, sizeof(buffer), 0);
		}
		fclose(fp);
	}

	return 0;
}

