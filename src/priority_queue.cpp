/**
 * @file      include/competition.h
 * @brief     Header file for competition
 * @author    Saurav Kumar
 * @author    Raja Srinivas
 * @author    Sanket Acharya
 * @author    Dinesh Kadirimangalam
 * @author    Preyash Parikh
 * @copyright 2020
 *
 **BSD 3-Clause License
 *
 *Copyright (c) 2020
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 *  contributors may be used to endorse or promote products derived from
 *  this software without specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <vector>
//#include <mutex>
#include <priority_queue.h>


PriorityQueue::PriorityQueue() {

}

PriorityQueue::~PriorityQueue() {

}

// Priority : 22 99 1 4  : 99 22 4 1
struct comparePriority
{
    bool operator()(OrderPart *lhs, OrderPart *rhs)
    {
        return (lhs->getPriority()) > (rhs->getPriority());
    }
} comp;


OrderPart* PriorityQueue::top() {
    sort();
    return priority_queue_.back();
}


void PriorityQueue::pop() {
    sort();
    priority_queue_.pop_back();
}

void PriorityQueue::push(OrderPart* part) {
    priority_queue_.push_back(part);
}

void PriorityQueue::sort() {
    std::sort(priority_queue_.begin(), priority_queue_.end(), comp);
}

bool PriorityQueue::empty() {
    return (priority_queue_.size() == 0);
}

void PriorityQueue::clear () {
    priority_queue_.clear();
}


void PriorityQueue::printPq(){

    sort();
    ROS_INFO_STREAM("-----------------");
  for (auto it = priority_queue_.rbegin(); it != priority_queue_.rend(); ++it) {
   ROS_INFO_STREAM ((*it)->getPartType() << " " << (*it)->getShipmentId());
  }
  ROS_INFO_STREAM("-----------------");
}

std::vector<OrderPart*>* PriorityQueue::getpq() {
 return &priority_queue_;
}

int PriorityQueue::size() {
	return priority_queue_.size();
}
