#ifndef _SINGLETON_BASE_HPP_
#define _SINGLETON_BASE_HPP_


#include <mutex>
#include <memory>

template<typename T>
class SingletonBase {
public:

    //获取、创建单例对象
    template<typename ...Args>
    static std::shared_ptr<T> GetInstance(Args&&... args) {
        if (m_instance == NULL) {
            std::lock_guard<std::mutex> lck(m_mutex);
            if (m_instance == NULL) {
                m_instance = std::make_shared<T>(std::forward<Args>(args)...);
            }
        }
        return m_instance;
    }

    //主动删除实例
    static void DeleteInstance() {
        if (m_instance) {
            m_instance.reset();
            m_instance = NULL;
        }
    }

private:
    explicit SingletonBase();
    SingletonBase(const SingletonBase&) = delete;
    SingletonBase& operator=(const SingletonBase&) = delete;
    ~SingletonBase();

private:
    static std::shared_ptr<T> m_instance;
    static std::mutex m_mutex;
};

template<typename T>
std::shared_ptr<T> SingletonBase<T>::m_instance = NULL;

template<typename T>
std::mutex SingletonBase<T>::m_mutex;

#endif


