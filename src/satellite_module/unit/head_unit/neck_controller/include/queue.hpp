/**
 * @file queue.hpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-10
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#ifndef QUEUE_HPP
#define QUEUE_HPP

template <class T>
class Queue {
public:
    Queue(int max_items = 10);
    ~Queue();

public:
    inline int length();
    inline int front();
    inline int back();
    inline bool is_max();

public:
    bool push(const T &item);
    T peek();
    T get(int index);
    T pop();
    void clear();

private:
    int _front;
    int _back;
    int _count;
    T *_data;
    int _max_items;
};

template <class T>
Queue<T>::Queue(int max_items)
{
    this->_front     = 0;
    this->_back      = 0;
    this->_count     = 0;
    this->_max_items = max_items;
    this->_data      = new T[max_items + 1];
}
template <class T>
Queue<T>::~Queue()
{
    delete[] this->_data;
}

template <class T>
inline int Queue<T>::length()
{
    return this->_count;
}

template <class T>
inline int Queue<T>::front()
{
    return this->_front;
}

template <class T>
inline int Queue<T>::back()
{
    return this->_back;
}
template <class T>
inline bool Queue<T>::is_max()
{
    return (this->_count >= this->_max_items);
}

template <class T>
bool Queue<T>::push(const T &item)
{
    bool result = true;
    if (false == this->is_max()) {
        this->_data[this->_back] = item;
        this->_back++;
        this->_count++;
        if (this->_back > this->_max_items) {
            this->_back -= (this->_max_items + 1);
        }
    } else {
        result = false;
    }
    return result;
}

template <class T>
T Queue<T>::pop()
{
    if (this->_count <= 0) {
        return T();
    } else {
        T result = this->_data[this->_front];
        this->_front++;
        this->_count--;

        if (this->_front > this->_max_items) {
            this->_front -= (this->_max_items + 1);
        }
        return result;
    }
}

template <class T>
T Queue<T>::peek()
{
    if (this->_count <= 0) {
        return T();
    } else {
        return this->_data[this->_front];
    }
}

template <class T>
T Queue<T>::get(int index)
{
    if ((this->_count <= 0) || (this->_count <= index)) {
        return T();
    } else {
        return this->_data[index];
    }
}

template <class T>
void Queue<T>::clear()
{
    this->_front = this->_back;
    this->_count = 0;
}

#endif
