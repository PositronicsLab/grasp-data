#ifndef _LOG_H_
#define _LOG_H_

//-----------------------------------------------------------------------------

#include <string>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

//-----------------------------------------------------------------------------
int __open( const char* path, int oflags, const mode_t& mode ) {
  return open( path, oflags, mode );
}

//-----------------------------------------------------------------------------
void __close( const int& fd ) {
  close( fd );
}

//-----------------------------------------------------------------------------
bool __write( int fd, const void* buffer, size_t bytes_to_write, ssize_t& bytes_written ) {
  bytes_written = write( fd, buffer, bytes_to_write );
  if( bytes_written == -1 ) {
    bytes_written = 0;
    return false;
/*
    if( errno == EAGAIN ) {
      return OS_ERROR_AGAIN;
    } else if( errno == EWOULDBLOCK ) {
      return OS_ERROR_AGAIN;
    } else if( errno == EBADF ) {
      return OS_ERROR_BADF;
    } else if( errno == EDESTADDRREQ ) {
      return OS_ERROR_DESTADDRREQ;
    } else if( errno == EDQUOT ) {
      return OS_ERROR_DQUOT;
    } else if( errno == EFAULT ) {
      return OS_ERROR_FAULT;
    } else if( errno == EFBIG ) {
      return OS_ERROR_FBIG;
    } else if( errno == EINVAL ) {
      return OS_ERROR_INVAL;
    } else if( errno == EINTR ) {
      return OS_ERROR_INTR;
    } else if( errno == EIO ) {
      return OS_ERROR_IO;
    } else if( errno == ENOSPC ) {
      return OS_ERROR_NOSPC;
    } else if( errno == EPIPE ) {
      return OS_ERROR_PIPE;
    }
*/
  }
  return true;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

class log_c {
private:
  bool _open;
  std::string _file;
  int _fd;
 
public:
  log_c( const std::string& file ) {
    _open = false;
    _file = file;
  }

  virtual ~log_c( void ) {}

  bool open( void ) {
    _fd = __open( _file.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH );

    if( _fd == -1 ) return false;
    return true;
  }

  void close( void ) {
    __close( _fd );
  }

  bool write( std::string data ) {
    ssize_t bytes_written;
    return __write( _fd, data.c_str(), data.size(), bytes_written );
  }

};

//-----------------------------------------------------------------------------

#endif // _LOG_H_

