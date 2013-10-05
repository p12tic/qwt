/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 1997   Josef Wilgen
 * Copyright (C) 2002   Uwe Rathmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#ifndef QWT_SCALE_MAP_TABLE_H
#define QWT_SCALE_MAP_TABLE_H

#include "qwt_global.h"
#include "qwt_scale_map.h"
#include "qwt_plot.h"
#include <qlist.h>

class QWT_EXPORT QwtScaleMapTable
{
public:
    bool isValid( int axisPos, int id ) const;
    const QwtScaleMap &map( int axisPos, int id ) const;

    QList< QwtScaleMap > maps[ QwtPlot::NumAxisPositions ];
};

inline bool QwtScaleMapTable::isValid( int axisPos, int id ) const
{
    if ( axisPos >= 0 && axisPos < QwtPlot::NumAxisPositions && id >= 0 )
        return maps[ axisPos ].size() > id;

    return false;
}

inline const QwtScaleMap &QwtScaleMapTable::map( int axisPos, int id ) const
{
    return maps[axisPos].at( id );
}

#endif
