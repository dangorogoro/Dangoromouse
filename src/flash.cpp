#include "mine.h"

// Flashから読みだしたデータを退避するRAM上の領域
// 4byteごとにアクセスをするので、アドレスが4の倍数になるように配置する
static uint8_t work_ram[BACKUP_FLASH_SECTOR_SIZE] __attribute__ ((aligned(4)));

// Flashのsector1の先頭に配置される変数(ラベル)
// 配置と定義はリンカスクリプトで行う

char _backup_flash_start;

void save_mazedata(Maze maze){
	for(int y = 0;y < MAZE_SIZE;y++){
		for(int x = 0;x < MAZE_SIZE;x++){
			work_ram[16 * y + x] = maze.getWall(x,y);
		}
	}
	Flash_write_back();
}
Maze upload_mazedata(){
	Flash_load();
	Maze tmpMaze;
	for(int y = 0;y < MAZE_SIZE;y++){
		for(int x = 0;x < MAZE_SIZE;x++){
			IndexVec vec(x,y);
			tmpMaze.updateWall(vec,work_ram[16 * y + x],true);
		}
	}

	return tmpMaze;
}

// Flashのsectoe1を消去
bool Flash_clear()
{
	FLASH_Unlock();
	FLASH_Status result = FLASH_EraseSector(BACKUP_FLASH_SECTOR_NUM, VoltageRange_3);
	FLASH_Lock();

	return result == FLASH_COMPLETE;
}

// Flashのsector1の内容を全てwork_ramに読み出す
// work_ramの先頭アドレスを返す
uint8_t* Flash_load()
{
	memcpy(work_ram, &_backup_flash_start, BACKUP_FLASH_SECTOR_SIZE);
	return work_ram;
}

// Flashのsector1を消去後、work_ramにあるデータを書き込む
bool Flash_write_back()
{
    // Flashをclear
	if (!Flash_clear()) return false;

	uint32_t *p_work_ram = (uint32_t*)work_ram;

	FLASH_Unlock();

    // work_ramにあるデータを4バイトごとまとめて書き込む
	FLASH_Status result;
	const size_t write_cnt = BACKUP_FLASH_SECTOR_SIZE / sizeof(uint32_t);
	for (size_t i=0;i<write_cnt;i++) {
		result = FLASH_ProgramWord(
					(uint32_t)(&_backup_flash_start)+sizeof(uint32_t)*i,
					p_work_ram[i]
				);
		if (result != FLASH_COMPLETE) break;
	}

	FLASH_Lock();

	return result == FLASH_COMPLETE;
}
