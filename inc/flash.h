#ifndef MY_FLASH_H
#define MY_FLASH_H
#define BACKUP_FLASH_SECTOR_NUM		FLASH_Sector_1
#define BACKUP_FLASH_SECTOR_SIZE	1024*16
extern char _backup_flash_start;
void save_mazedata(Maze maze);
Maze upload_mazedata();
bool Flash_clear();
uint8_t* Flash_load();
bool Flash_write_back();
#endif
