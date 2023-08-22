#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define WIDTH 60
#define HIGH 20

typedef struct {
  int x;
  int y;
} Body;

typedef struct {
  Body list[WIDTH * HIGH];
  int size;
  Body food;
} Snake;

void InitFood(Body* food) {
  srand(time(NULL));
  food->x = rand() % WIDTH;
  food->y = rand() % HIGH;
}

void InitSnake(Snake* snake) {
  snake->list[0].x = WIDTH / 2;
  snake->list[0].y = HIGH / 2;
  snake->list[1].x = WIDTH / 2 - 1;
  snake->list[1].y = HIGH / 2 - 1;
  snake->size = 2;
  //设置食物位置
  InitFood(&(snake->food));
}

void ShowUI(Snake* snake) {
    for (int i = 0; i < snake->size; ++i) {
        if (i == 0) {
            printf("@");
        } else {
            printf("*");
        }
    }
}

void ShowWall() {
  for (int i = 0; i <= HIGH; ++i) {
    for (int j = 0; j <= WIDTH; ++j) {
      if (i == 0 || j == 0 || i == HIGH || j == WIDTH) {
        printf("+");
      } else {
        printf(" ");
      }
    }
    printf("\n");
  }
}

int main() {
  Snake* snake = (Snake*)malloc(sizeof(Snake));
  InitSnake(snake);
  ShowWall();
  ShowUI(snake);

  free(snake);
  return 0;
}