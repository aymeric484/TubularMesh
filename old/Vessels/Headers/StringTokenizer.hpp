#ifndef STRINGTOKENIZER_H
#define STRINGTOKENIZER_H

#include <string>
using namespace std;

class StringTokenizer
{
private:
	string str;
	string delim;
	string::size_type index;
	bool hasTokens;

	void decalToNextToken()
	{
		bool encore=true;
		while (index<str.length() && encore)
		{
			char cur=str.at(index);
			string::size_type pos=delim.find(cur,0);
			if (pos==string::npos)
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
			string::size_type pos=delim.find(cur,0);
			if (pos!=string::npos)
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
	StringTokenizer(string s, string del)
	{
		str=string(s);
		delim=string(del);
		index=0;
		decalToNextToken();
	}

	bool hasMoreTokens()
	{
		return index<str.length();
	}

	string nextToken()
	{
		string::size_type idStart=index;
		decalToEndOfToken();

		string::size_type idStop=index;
		decalToNextToken();
		return string(str,idStart,idStop-idStart);
	}

};

#endif
