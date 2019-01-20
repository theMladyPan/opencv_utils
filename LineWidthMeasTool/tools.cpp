#include "tools.h"


QStringList getQStringList(const std::vector<std::string> &strVector)
{
    QStringList out;
    for(auto str:strVector){
        out.append(QString::fromStdString(str));
    }
    return out;
}
