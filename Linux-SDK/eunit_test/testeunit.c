#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/usb/video.h>
#include <linux/videodev2.h>

// UVC扩展单元控制结构
struct uvc_xu_control {
    __u8 unit;
    __u8 selector;
    __u16 size;
    __u8 *data;
};

// UVC扩展单元查询结构
struct uvc_xu_control_query {
    __u8 unit;
    __u8 selector;
    __u8 query;
    __u16 size;
    __u8 *data;
};

#define UVCIOC_CTRL_QUERY _IOWR('u', 0x21, struct uvc_xu_control_query)

int main(int argc, char *argv[]) {
    int fd;
    const char *device = "/dev/video4";
    __u8 unit_id = 3;
    __u8 cs;
    __u8 len;
    int ret;
    
    // 检查参数
    if (argc < 2) {
        fprintf(stderr, "用法: %s <CS值(十进制)>\n", argv[0]);
        fprintf(stderr, "示例: %s 1\n", argv[0]);
        return 1;
    }
    
    cs = (__u8)atoi(argv[1]);
    
    // 打开设备
    fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("打开设备失败");
        return 1;
    }
    
    // 准备GET_LEN查询
    struct uvc_xu_control_query query;
    memset(&query, 0, sizeof(query));
    
    query.unit = unit_id;
    query.selector = cs;
    query.query = UVC_GET_LEN;  // 值为0x86
    query.size = 2;             // 长度用1字节
    query.data = &len;
    
    printf("正在查询: device=%s, unit=%d, cs=%d, query=GET_LEN(0x86)\n", 
           device, unit_id, cs);
    
    // 执行ioctl
    ret = ioctl(fd, UVCIOC_CTRL_QUERY, &query);
    if (ret < 0) {
        perror("ioctl查询失败");
        close(fd);
        return 1;
    }
    
    // 显示结果
    printf("GET_LEN 成功!\n");
    printf("控制值长度: %d 字节 (0x%02x)\n", len, len);
    
    close(fd);
    return 0;
}