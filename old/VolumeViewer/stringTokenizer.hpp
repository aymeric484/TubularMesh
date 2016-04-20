#ifndef STRINGTOKENIZER_H
#define STRINGTOKENIZER_H

#include <string>

namespace CGoGN
{

class StringTokenizer
{
private:
	std::string str;
	std::string delim;
	std::string::size_type index;
	bool hasTokens;

	void decalToNextToken()
	{
		bool encore=true;
		while (index<str.length() && encore)
		{
			char cur=str.at(index);
			std::string::size_type pos=delim.find(cur,0);
			if (pos==std::string::npos)
			{
				encore=false;
			}
			else
			{
				index++;
			}
		}
	}

	void decalToEndOfToken()
	{
		bool encore=true;
		while (index<str.length() && encore)
		{
			char cur=str.at(index);
			std::string::size_type pos=delim.find(cur,0);
			if (pos!=std::string::npos)
			{
				encore=false;
			}
			else
			{
				index++;
			}
		}
	}
public:
	StringTokenizer(std::string s, std::string del)
	{
		str=std::string(s);
		delim=std::string(del);
		index=0;
		decalToNextToken();
	}

	bool hasMoreTokens()
	{
		return index<str.length();
	}

	std::string nextToken()
	{
		std::string::size_type idStart=index;
		decalToEndOfToken();

		std::string::size_type idStop=index;
		decalToNextToken();

		return std::string(str,idStart,idStop-idStart);
	}

};

}

#endif
