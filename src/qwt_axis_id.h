/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 1997   Josef Wilgen
 * Copyright (C) 2002   Uwe Rathmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#ifndef QWT_AXIS_ID_H
#define QWT_AXIS_ID_H

#include "qwt_global.h"

#ifndef QT_NO_DEBUG_STREAM
#include <qdebug.h>
#endif

class QWT_EXPORT QwtAxisId
{
public:
    QwtAxisId( int position, int index = 0 );

    bool operator==( const QwtAxisId & ) const;
    bool operator!=( const QwtAxisId & ) const;

    int pos;
    int id;
};

inline QwtAxisId::QwtAxisId( int position, int index ):
    pos( position ),
    id( index )
{
}

inline bool QwtAxisId::operator==( const QwtAxisId &other ) const
{
    return ( pos == other.pos ) && ( id == other.id );
}

inline bool QwtAxisId::operator!=( const QwtAxisId &other ) const
{
    return !operator==( other );
}

#ifndef QT_NO_DEBUG_STREAM
QWT_EXPORT QDebug operator<<( QDebug, const QwtAxisId & );
#endif

#endif
