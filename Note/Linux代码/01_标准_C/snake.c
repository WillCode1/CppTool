/*
   贪吃蛇练习
   */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#define     APPLE             '@'
#define     SNAKE_HEAD        '+'
#define     SNAKE_TAIL        '-'
typedef struct {
	int row, col;
} pt;
typedef struct {
	pt head, tail;
} snake;
enum {NOOVERLAP, OVERLAP};
enum {UP, DOWN, LEFT, RIGHT};
//初始化蛇的位置
void init_snake(snake *p_snake) {
    p_snake->head.row = rand() % (SIZE - 2) + 2;
	p_snake->head.col = rand() % (SIZE - 2) + 2;
	switch(rand() % 4) {
	case UP:
		p_snake->tail.row = p_snake->head.row - 1;
		p_snake->tail.col = p_snake->head.col;
		break;
	case DOWN:
		p_snake->tail.row = p_snake->head.row + 1;
		p_snake->tail.col = p_snake->head.col;
		break;
	case LEFT:
		p_snake->tail.row = p_snake->head.row;
		p_snake->tail.col = p_snake->head.col - 1;
		break;
	case RIGHT:
		p_snake->tail.row = p_snake->head.row;
		p_snake->tail.col = p_snake->head.col + 1;
		break;
	}
}
//初始化苹果的位置
void init_apple(pt *p_apple) {
	p_apple->row = rand() % SIZE + 1;
	p_apple->col = rand() % SIZE + 1;
}
//显示整个地图
void show_map(const snake *p_snake, const pt *p_apple) {
    int row = 0, col = 0;
	for (row = 1;row <= SIZE;row++) {
		for (col = 1;col <= SIZE;col++) {
			if (row == p_snake->head.row && col == p_snake->head.col) {
				printf("%c", SNAKE_HEAD);
			}
			else if (row == p_snake->tail.row && col == p_snake->tail.col) {
				printf("%c", SNAKE_TAIL);
			}
			else if (row == p_apple->row && col == p_apple->col) {
				printf("%c", APPLE);
			}
			else {
				printf(" ");
			}
		}
		printf("\n");
	}
}
//把蛇的位置移动一步
void move_snake(snake *p_snake){
	int direction = 0;
	pt tmp = {};
	printf("请输入代表移动方向的数字，%d代表上，%d代表下，%d代表左，%d代表右\n", UP, DOWN, LEFT, RIGHT);
	scanf("%d", &direction);
	direction = direction >= 0 ? direction : 0 - direction;
	direction = direction % 4;
	tmp = p_snake->head;
	switch(direction) {
	case UP:
		tmp.row--;
		break;
	case DOWN:
		tmp.row++;
		break;
	case LEFT:
		tmp.col--;
		break;
	case RIGHT:
		tmp.col++;
		break;
	}
	if (tmp.row < 1 || tmp.row > SIZE || tmp.col < 1 || tmp.col > SIZE) {
		return;
	}
	if (tmp.row == p_snake->tail.row && tmp.col == p_snake->tail.col) {
		return; 
	}
	p_snake->tail = p_snake->head;
	p_snake->head = tmp;
}
//判断蛇和苹果的位置是否重叠
int overlap(const snake *p_snake, const pt *p_apple) {
	if (p_snake->head.row == p_apple->row && p_snake->head.col == p_apple->col) {
		return OVERLAP;
	}
	if (p_snake->tail.row == p_apple->row && p_snake->tail.col == p_apple->col) {
		return OVERLAP;
	}
	return NOOVERLAP;
}
int main() {
	snake snake1 = {};
	pt apple = {};
	srand(time(0));
	init_snake(&snake1);
	init_apple(&apple);
	while (1) {
	    int ret = overlap(&snake1, &apple);
		if (ret == NOOVERLAP) {break;
		}else {
			init_apple(&apple);
		}
	}
    show_map(&snake1, &apple);
	while (1) {
		move_snake(&snake1);
	    while (1) {
			int ret = overlap(&snake1, &apple);
			if (ret == NOOVERLAP) {break;
			}else {
				init_apple(&apple);
			}
		}
		show_map(&snake1, &apple);
	}
	return 0;
}




