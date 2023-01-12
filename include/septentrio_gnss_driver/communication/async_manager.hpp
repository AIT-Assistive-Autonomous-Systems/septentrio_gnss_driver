// *****************************************************************************
//
// © Copyright 2020, Septentrio NV/SA.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//    3. Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

// *****************************************************************************
//
// Boost Software License - Version 1.0 - August 17th, 2003
//
// Permission is hereby granted, free of charge, to any person or organization
// obtaining a copy of the software and accompanying documentation covered by
// this license (the "Software") to use, reproduce, display, distribute,
// execute, and transmit the Software, and to prepare derivative works of the
// Software, and to permit third-parties to whom the Software is furnished to
// do so, all subject to the following:

// The copyright notices in the Software and this entire statement, including
// the above license grant, this restriction and the following disclaimer,
// must be included in all copies of the Software, in whole or in part, and
// all derivative works of the Software, unless such copies or derivative
// works are solely in the form of machine-executable object code generated by
// a source language processor.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
// SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
// FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
// *****************************************************************************

#pragma once

// Boost includes
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/regex.hpp>

// ROSaic includes
#include <septentrio_gnss_driver/crc/crc.h>
#include <septentrio_gnss_driver/parsers/parsing_utilities.h>

// local includes
#include <septentrio_gnss_driver/communication/io.hpp>
#include <septentrio_gnss_driver/communication/telegram.hpp>

/**
 * @file async_manager.hpp
 * @date 20/08/20
 * @brief Implements asynchronous operations for an I/O manager
 *
 * Such operations include reading NMEA messages and SBF blocks yet also sending
 * commands to serial port or via TCP/IP.
 */

namespace io {

    /**
     * @class AsyncManagerBase
     * @brief Interface (in C++ terms), that could be used for any I/O manager,
     * synchronous and asynchronous alike
     */
    class AsyncManagerBase
    {
    public:
        virtual ~AsyncManagerBase() {}
        //! Sends commands to the receiver
        virtual bool send(const std::string& cmd) = 0;
        //! Connects the stream
        [[nodiscard]] virtual bool connect() = 0;
    };

    /**
     * @class AsyncManager
     * @brief This is the central interface between ROSaic and the Rx(s), managing
     * I/O operations such as reading messages and sending commands..
     *
     * SocketT is either boost::asio::serial_port or boost::asio::tcp::ip
     */
    template <typename SocketT>
    class AsyncManager : public AsyncManagerBase
    {
    public:
        /**
         * @brief Class constructor
         * @param[in] node Pointer to node
         * @param[in] telegramQueue Telegram queue
         */
        AsyncManager(ROSaicNodeBase* node, TelegramQueue* telegramQueue) :
            node_(node), ioSocket_(node, &ioService_), telegramQueue_(telegramQueue)
        {
        }

        ~AsyncManager()
        {
            running_ = false;
            flushOutputQueue();
            close();
            node_->log(LogLevel::DEBUG, "AsyncManager shutting down threads");
            ioService_.stop();
            ioThread_.join();
            watchdogThread_.join();
            node_->log(LogLevel::DEBUG, "AsyncManager threads stopped");
        }

        [[nodiscard]] bool connect()
        {
            if (ioSocket_.connect())
            {
                return false;
            }
            receive();

            watchdogThread_ = std::thread(std::bind(&TcpClient::runWatchdog, this));
            return true;
        }

        void send(const std::string& cmd)
        {
            if (cmd.size() == 0)
            {
                node_->log(
                    LogLevel::ERROR,
                    "AsyncManager message size to be sent to the Rx would be 0");
            }

            ioService_.post(boost::bind(&AsyncManager<SocketT>::write, this, cmd));
        }

    private:
        void receive()
        {
            resync();
            ioThread_ = std::thread(
                std::thread(std::bind(&AsyncManager::runIoService, this)));
        }

        void flushOutputQueue()
        {
            ioService_.post([this]() { write(); });
        }

        void close()
        {
            ioService_.post([this]() { socket_->close(); });
        }

        void runIoService()
        {
            ioService_.run();
            node_->log(LogLevel::DEBUG, "AsyncManager ioService terminated.");
        }

        void runWatchdog()
        {
            while (running_)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10000));

                if (running_ && ioService_.stopped())
                {
                    node_->log(LogLevel::DEBUG,
                               "AsyncManager connection lost. Trying to reconnect.");
                    ioService_.reset();
                    ioThread_->join();
                    receive();
                }
            }
        }

        void write(const std::string& cmd)
        {
            boost::asio::write(ioSocket_.stream(),
                               boost::asio::buffer(cmd.data(), cmd.size()));
            // Prints the data that was sent
            node_->log(LogLevel::DEBUG, "AsyncManager sent the following " +
                                            std::to_string(cmd.size()) +
                                            " bytes to the Rx: " + cmd);
        }

        void resync()
        {
            message_.reset(new Telegram);
            readSync<0>();
        }

        template <uint8_t index>
        void readSync()
        {
            static_assert(index < 3);

            boost::asio::async_read(
                ioSocket_.stream(),
                boost::asio::buffer(message_->data.data() + index, 1),
                [this](boost::system::error_code ec, std::size_t numBytes) {
                    Timestamp stamp = node_->getTime();

                    if (!ec)
                    {
                        if (numBytes == 1)
                        {
                            std::byte& currByte = message_->data[index];

                            if (currByte == SYNC_0)
                            {
                                message_->stamp = stamp;
                                readSync<1>();
                            } else
                            {
                                switch (index)
                                {
                                case 0:
                                {
                                    if ((currByte == CONNECTION_DESCRIPTOR_BYTE_I) ||
                                        (currByte == CONNECTION_DESCRIPTOR_BYTE_C) ||
                                        (currByte == CONNECTION_DESCRIPTOR_BYTE_U) ||
                                        (currByte == CONNECTION_DESCRIPTOR_BYTE_N) ||
                                        (currByte == CONNECTION_DESCRIPTOR_BYTE_D))
                                    {
                                        message_->type = CONNECTION_DESCRIPTOR;
                                        readCd();
                                    } else
                                        readSync<0>();
                                    break;
                                }
                                case 1:
                                {
                                    switch (currByte)
                                    {
                                    case SBF_SYNC_BYTE_1:
                                    {
                                        message_->type = SBF;
                                        readSbfHeader();
                                        break;
                                    }
                                    case NMEA_SYNC_BYTE_1:
                                    {
                                        message_->type = NMEA;
                                        readSync<2>();
                                        break;
                                    }
                                    case RESPONSE_SYNC_BYTE_1:
                                    {
                                        message_->type = RESPONSE;
                                        readSync<2>();
                                        break;
                                    }
                                    default:
                                    {
                                        node_->log(
                                            LogLevel::DEBUG,
                                            "AsyncManager sync 1 read fault, should never come here.");
                                        resync();
                                        break;
                                    }
                                    }
                                    break;
                                }
                                case 2:
                                {
                                    switch (currByte)
                                    {
                                    case NMEA_SYNC_BYTE_2:
                                    {
                                        if (message_->type == NMEA)
                                            readString();
                                        else
                                            resync();
                                        break;
                                    }
                                    case ERROR_SYNC_BYTE_2:
                                    {
                                        if (message_->type == ERROR)
                                            readString();
                                        else
                                            resync();
                                        break;
                                    }
                                    case RESPONSE_SYNC_BYTE_2:
                                    {
                                        if (message_->type == RESPONSE)
                                            readString();
                                        else
                                            resync();
                                        break;
                                    }
                                    default:
                                    {
                                        node_->log(
                                            LogLevel::DEBUG,
                                            "AsyncManager sync 2 read fault, should never come here.");
                                        resync();
                                        break;
                                    }
                                    }
                                    break;
                                }
                                default:
                                {
                                    node_->log(
                                        LogLevel::DEBUG,
                                        "AsyncManager sync read fault, should never come here.");
                                    resync();
                                    break;
                                }
                                }
                            }
                        } else
                        {
                            node_->log(
                                LogLevel::DEBUG,
                                "AsyncManager sync read fault, wrong number of bytes read: " +
                                    std::to_string(numBytes));
                            resync();
                        }
                    } else
                    {
                        node_->log(LogLevel::DEBUG,
                                   "AsyncManager sync read error: " + ec.message());
                        resync();
                    }
                });
        }

        void readSbfHeader()
        {
            message_->data.resize(SBF_HEADER_SIZE);

            boost::asio::async_read(
                ioSocket_.stream(),
                boost::asio::buffer(message_->data.data() + 2, SBF_HEADER_SIZE - 2),
                [this](boost::system::error_code ec, std::size_t numBytes) {
                    if (!ec)
                    {
                        if (numBytes == (SBF_HEADER_SIZE - 2))
                        {
                            unit16_t length =
                                parsing_utilities::getLength(message_->data.data());
                            if (length > MAX_SBF_SIZE)
                            {
                                node_->log(
                                    LogLevel::DEBUG,
                                    "AsyncManager SBF header read fault, length of block exceeds " +
                                        std::to_string(MAX_SBF_SIZE) + ": " +
                                        std::to_string(length));
                            } else
                                readSbf(length);
                        } else
                        {
                            node_->log(
                                LogLevel::DEBUG,
                                "AsyncManager SBF header read fault, wrong number of bytes read: " +
                                    std::to_string(numBytes));
                            resync();
                        }
                    } else
                    {
                        node_->log(LogLevel::DEBUG,
                                   "AsyncManager SBF header read error: " +
                                       ec.message());
                        resync();
                    }
                });
        }

        void readSbf(std::size_t length)
        {
            message_->data.resize(length);

            boost::asio::async_read(
                ioSocket_.stream(),
                boost::asio::buffer(message_->data.data() + SBF_HEADER_SIZE,
                                    length - SBF_HEADER_SIZE),
                [this](boost::system::error_code ec, std::size_t numBytes) {
                    if (!ec)
                    {
                        if (numBytes == (length - SBF_HEADER_SIZE))
                        {
                            if (crc::isValid(message_->data.data()))
                            {
                                message_->sbfId =
                                    parsing_utilities::getId(message_->data.data());

                                telegramQueue_->push(message_);
                            }
                        } else
                        {
                            node_->log(
                                LogLevel::DEBUG,
                                "AsyncManager SBF read fault, wrong number of bytes read: " +
                                    std::to_string(numBytes));
                        }
                    } else
                    {
                        node_->log(LogLevel::DEBUG,
                                   "AsyncManager SBF read error: " + ec.message());
                    }
                    resync();
                });
        }

        void readCd()
        {
            message_->data.resize(1);
            readStringElements();
        }

        void readString()
        {
            message_->data.resize(3);
            readStringElements();
        }

        void readStringElements()
        {
            std::array<std::byte, 1> buf;

            boost::asio::async_read(
                ioSocket_.stream(), boost::asio::buffer(buf.data(), 1),
                [this](boost::system::error_code ec, std::size_t numBytes) {
                    if (!ec)
                    {
                        if (numBytes == 1)
                        {
                            message_->data.push_back(buf[0]);
                            switch (buf[0])
                            {
                            case SYNC_0:
                            {
                                message_.reset(new Telegram);
                                message_->data[0] = buf[0];
                                message_->stamp = node_->getTime();
                                node_->log(
                                    LogLevel::DEBUG,
                                    "AsyncManager string read fault, sync 0 found."));
                                readSync<1>();
                                break;
                            }
                            case LF:
                            {
                                if (message_->data[message_->data.size() - 2] == CR)
                                    telegramQueue_->push(message_);
                                resync();
                                break;
                            }
                            case CONNECTION_DESCRIPTOR_FOOTER:
                            {
                                if (message_->type == CONNECTION_DESCRIPTOR)
                                    telegramQueue_->push(message_);
                                resync();
                                break;
                            }
                            default:
                            {
                                readString();
                                break;
                            }
                            }
                        } else
                        {
                            node_->log(
                                LogLevel::DEBUG,
                                "AsyncManager string read fault, wrong number of bytes read: " +
                                    std::to_string(numBytes));
                            resync();
                        }
                    } else
                    {
                        node_->log(LogLevel::DEBUG,
                                   "AsyncManager string read error: " +
                                       ec.message());
                        resync();
                    }
                });
        }

        //! Pointer to the node
        ROSaicNodeBase* node_;
        SocketT ioSocket_;
        std::atomic<bool> running_;
        boost::asio::io_service ioService_;
        std::thread ioThread_;
        std::thread watchdogThread_;
        //! Timestamp of receiving buffer
        Timestamp recvStamp_;
        //! Telegram
        std::shared_ptr<Telegram> message_;
        //! TelegramQueue
        TelegramQueue* telegramQueue_;
    };
} // namespace io