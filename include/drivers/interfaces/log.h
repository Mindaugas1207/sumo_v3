#ifndef LOG_H
#define LOG_H

/*! @brief Interface for logging messages.
 */
class Log {
public:
    /*! @brief Log an informational message.
     * @param message The message to log.
     */
    virtual void info(const char* message) = 0;

    /*! @brief Log a warning message.
     * @param message The message to log.
     */
    virtual void warning(const char* message) = 0;

    /*! @brief Log an error message.
     * @param message The message to log.
     */
    virtual void error(const char* message) = 0;

    /*! @brief Log a debug message.
     * @param message The message to log.
     */
    virtual void debug(const char* message) = 0;

    /*! @brief Set the log level.
     * @param level The log level to set.
     */
    virtual void setLogLevel(int level) = 0;

    /*! @brief Get the current log level.
     * @return The current log level.
     */
    virtual int getLogLevel() = 0;

    /*! @brief Get the name of a log level.
     * @param level The log level.
     * @return The name of the log level.
     */
    virtual const char* getLogLevelName(int level) = 0;
};

#endif // LOG_H