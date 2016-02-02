/**
 * @file   sinsmsg.h
 * @brief  Serialization helpers 
 * @author Venkat Rajagopalan
 * @date   10/2015
 */
/**************************************************************************
 * Copyright (c) 2015, Carnegie Mellon University                         *
 *                                                                        *
 * DO NOT DISTRIBUTE WITHOUT PRIOR WRITTEN PERMISSION                     *
 *                                                                        *
 * THE MATERIAL EMBODIED IN THIS SOFTWARE IS PROVIDED TO YOU "AS-IS"      *
 * AND WITHOUT WARRANTY OF ANY KIND, EXPRESS, IMPLIED OR OTHERWISE,       *
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY OR       *
 * FITNESS FOR A PARTICULAR PURPOSE.  IN NO EVENT SHALL CARNEGIE MELLON   *
 * UNIVERSITY BE LIABLE TO YOU OR ANYONE ELSE FOR ANY DIRECT,             *
 * SPECIAL, INCIDENTAL, INDIRECT OR CONSEQUENTIAL DAMAGES OF ANY          *
 * KIND, OR ANY DAMAGES WHATSOEVER, INCLUDING WITHOUT LIMITATION,         *
 * LOSS OF PROFIT, LOSS OF USE, SAVINGS OR REVENUE, OR THE CLAIMS OF      *
 * THIRD PARTIES, WHETHER OR NOT CARNEGIE MELLON UNIVERSITY HAS BEEN      *
 * ADVISED OF THE POSSIBILITY OF SUCH LOSS, HOWEVER CAUSED AND ON         *
 * ANY THEORY OF LIABILITY, ARISING OUT OF OR IN CONNECTION WITH THE      *
 * POSSESSION, USE OR PERFORMANCE OF THIS SOFTWARE.                       *
 **************************************************************************/

#include "sinsmsg.h"

#include <sstream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

namespace BASIL
{
  namespace INS
  {
    // serialize InsMsg 
    bool toString(const InsMsg& msg, std::string& data)
    {
      std::ostringstream oss;
      boost::archive::binary_oarchive oar(oss, boost::archive::no_header);
      
      try
        {
          oar << msg;
        }
      catch(const boost::archive::archive_exception& e)
        {
          return false;
        }
      
      data = oss.str();
      return true;
    }

    // de-serialize InsMsg
    bool fromString(const std::string& data, InsMsg& msg)
    {
      if(data.empty())
        {
          return false;
        }

      std::istringstream iss(data);
      boost::archive::binary_iarchive iar(iss, boost::archive::no_header);

      try
        {
          iar >> msg;
        }
      catch(const boost::archive::archive_exception& e)
        {
          return false;
        }

      return true;
    }
  }
}
