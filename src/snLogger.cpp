/****************************************************************************/
/*Copyright (c) 2014, Florent DEVILLE.                                      */
/*All rights reserved.                                                      */
/*                                                                          */
/*Redistribution and use in source and binary forms, with or without        */
/*modification, are permitted provided that the following conditions        */
/*are met:                                                                  */
/*                                                                          */
/* - Redistributions of source code must retain the above copyright         */
/*notice, this list of conditions and the following disclaimer.             */
/* - Redistributions in binary form must reproduce the above                */
/*copyright notice, this list of conditions and the following               */
/*disclaimer in the documentation and/or other materials provided           */
/*with the distribution.                                                    */
/* - The names of its contributors cannot be used to endorse or promote     */
/*products derived from this software without specific prior written        */
/*permission.                                                               */
/* - The source code cannot be used for commercial purposes without         */
/*its contributors' permission.                                             */
/*                                                                          */
/*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       */
/*"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         */
/*LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         */
/*FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE            */
/*COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,       */
/*INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,      */
/*BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;          */
/*LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER          */
/*CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT        */
/*LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN         */
/*ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           */
/*POSSIBILITY OF SUCH DAMAGE.                                               */
/****************************************************************************/
#include "snLogger.h"

#include "log4cpp/Category.hh"
#include "log4cpp/Appender.hh"
#include "log4cpp/FileAppender.hh"

using namespace Supernova::Vector;

namespace Supernova
{
	snLogger* snLogger::m_instance = 0;

	snLogger* snLogger::getInstance()
	{
		if (m_instance == 0)
		{
			m_instance = new snLogger();
		}
		return m_instance;
	}

	void snLogger::initialize()
	{
		//create a file appender
		log4cpp::Appender* appender = new log4cpp::FileAppender("default", "supernova_output.log", false, _S_IREAD | _S_IWRITE);
		appender->setLayout(new log4cpp::BasicLayout());

		log4cpp::Category& root = log4cpp::Category::getRoot();
		root.addAppender(appender);
	}

	void snLogger::shutdown()
	{
		if (m_instance != 0)
		{
			delete m_instance;
			m_instance = 0;
		}
	}

	void snLogger::logInfo(string _msg)
	{
		log4cpp::Category::getRoot().info(_msg);
	}

	void snLogger::logError(string _msg)
	{
		log4cpp::Category::getRoot().error(_msg);
	}

	void snLogger::logWarn(string _msg)
	{
		log4cpp::Category::getRoot().warn(_msg);
	}

	string snLogger::toString(const snVec& _v) const
	{
		return std::to_string(snVec4GetX(_v)) + ", " + std::to_string(snVec4GetY(_v)) + ", " + std::to_string(snVec4GetZ(_v)) +
			", " + std::to_string(snVec4GetW(_v));
	}

	string snLogger::toString(unsigned int _u) const
	{
		return std::to_string(_u);
	}

	snLogger::snLogger()
	{
	}

	snLogger::~snLogger()
	{
	}
}