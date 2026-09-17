#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <utility>
#include <vector>

#include "TestTime.h"
#include "follow/runtime/EventQueue.h"
#include "follow/runtime/Latest.h"

using namespace follow::runtime;
using follow::test::at;
using namespace std::chrono_literals;

TEST(LatestTest, EmptyUntilWritten)
{
  Latest<int> slot;
  EXPECT_FALSE(slot.read());
  EXPECT_FALSE(slot.readFresh(at(1.0), 1s));
}

TEST(LatestTest, ReadReturnsNewestValueStampAndSequence)
{
  // Setup
  Latest<int> slot;

  // Run
  slot.write(7, at(1.0));
  slot.write(8, at(1.1));
  auto stamped = slot.read();

  // Assert
  ASSERT_TRUE(stamped);
  EXPECT_EQ(stamped->value, 8);
  EXPECT_EQ(stamped->t, at(1.1));
  EXPECT_EQ(stamped->sequence, 2u);
}

TEST(LatestTest, ReadFreshRejectsValuesOlderThanMaxAge)
{
  Latest<int> slot;
  slot.write(5, at(1.0));

  EXPECT_EQ(slot.readFresh(at(1.2), 200ms), 5);
  EXPECT_FALSE(slot.readFresh(at(1.201), 200ms));
}

TEST(LatestTest, ConcurrentReadersNeverSeeTornValues)
{
  // Setup: the writer always stores a pair of equal numbers
  Latest<std::pair<int, int>> slot;
  std::atomic<bool> done{false};
  std::atomic<int> torn{0};

  // Run
  std::thread reader([&] {
    while (!done) {
      if (auto stamped = slot.read(); stamped && stamped->value.first != stamped->value.second) {
        ++torn;
      }
    }
  });
  for (int i = 0; i < 20000; ++i) {
    slot.write({i, i}, at(0.0));
  }
  done = true;
  reader.join();

  // Assert
  EXPECT_EQ(torn, 0);
  EXPECT_EQ(slot.read()->sequence, 20000u);
}

TEST(EventQueueTest, DrainReturnsEventsOldestFirstAndEmptiesTheQueue)
{
  EventQueue<int> queue;
  queue.push(1);
  queue.push(2);
  queue.push(3);

  EXPECT_EQ(queue.drain(), (std::vector<int>{1, 2, 3}));
  EXPECT_TRUE(queue.drain().empty());
}

TEST(EventQueueTest, NoEventIsLostBetweenThreads)
{
  // Setup
  EventQueue<int> queue;
  std::vector<int> received;
  std::atomic<bool> producerDone{false};

  // Run
  std::thread producer([&] {
    for (int i = 0; i < 20000; ++i) {
      queue.push(i);
    }
    producerDone = true;
  });
  while (!producerDone || received.size() < 20000) {
    for (int event : queue.drain()) {
      received.push_back(event);
    }
  }
  producer.join();

  // Assert
  ASSERT_EQ(received.size(), 20000u);
  for (int i = 0; i < 20000; ++i) {
    ASSERT_EQ(received[i], i);
  }
}
