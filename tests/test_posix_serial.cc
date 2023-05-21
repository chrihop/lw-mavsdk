#include <gtest/gtest.h>
#include "lwmavsdk.h"
#include <pty.h>
#include <vector>
#include <numeric>

static int create_mock_pts(const char * link)
{
    int master, slave;
    char pts[256];

    if (openpty(&master, &slave, pts, nullptr, nullptr) < 0)
    {
        perror("openpty");
        exit(1);
    }

    if (symlink(pts, link) < 0)
    {
        perror("symlink");
        exit(1);
    }

    return master;
}

static void destroy_mock_pts(int master, const char * link)
{
    close(master);
    unlink(link);
}

int master_a, master_b;
const char * device_a = "./uart-a";
const char * device_b = "./uart-b";

static lwm_vehicle_t vehicle;

class PosixSerialTest : public ::testing::Test
{
public:
    void SetUp() override
    {
        master_a = create_mock_pts(device_a);
        master_b = create_mock_pts(device_b);

        lwm_vehicle_init(&vehicle);
        lwm_conn_open(&vehicle.conn, LWM_CONN_TYPE_SERIAL, device_a, 115200);
    }
    void TearDown() override
    {
        destroy_mock_pts(master_a, device_a);
        destroy_mock_pts(master_b, device_b);
    }
};


TEST_F(PosixSerialTest, posix_serial_attach_reader)
{
    std::vector<uint64_t> data(512);
    std::iota(data.begin(), data.end(), 0);
    write(master_a, data.data(), data.size() * sizeof(uint64_t));

    std::vector<uint64_t> buffer(512);
    vehicle.conn.recv(&vehicle.conn, (uint8_t *) buffer.data(), buffer.size() * sizeof(uint64_t));
    for (int i = 0; i < data.size(); i++)
    {
        if (data[i] != buffer[i])
        {
            printf("data[%d] = %lu, buffer[%d] = %lu\n", i, data[i], i, buffer[i]);
            FAIL();
        }
    }
    ASSERT_EQ(data, buffer);
}
