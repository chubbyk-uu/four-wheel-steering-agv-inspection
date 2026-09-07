#include "agv_linescan/queue_budget.hpp"
#include <gtest/gtest.h>
using namespace agv_linescan;
TEST(QueueBudget, VariableRateCapacityAndRecovery) {
 QueueBudget q(100,8,.1);q.Push(80,0);q.Push(20,.01);
 EXPECT_THROW(q.Push(1,.02),std::runtime_error);EXPECT_EQ(q.lines,100u);
 q.Pop(.03);q.Push(70,.04);EXPECT_EQ(q.lines,90u);EXPECT_EQ(q.peakLines,100u);
 q.Clear();q.Push(100,1);q.Pop(1.05);EXPECT_EQ(q.lines,0u);
}
TEST(QueueBudget, DeadlineAndTaskLimitAreIndependent) {
 QueueBudget q(10000,2,.05);q.Push(1,0);q.Push(1,.01);
 EXPECT_THROW(q.Push(1,.02),std::runtime_error);
 EXPECT_THROW(q.Pop(.051),std::runtime_error);EXPECT_EQ(q.lines,2u);
 q.Clear();q.Push(1000,1);q.Pop(1.01);EXPECT_EQ(q.Jobs(),0u);
}
