/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 1997   Josef Wilgen
 * Copyright (C) 2002   Uwe Rathmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#include "qwt_plot.h"
#include "qwt_axes_mask.h"
#include <qlist.h>
#include <qalgorithms.h>

class QwtAxesMask::PrivateData
{
public:
    QList<int> disabledAxes[ QwtPlot::NumAxisPositions ];
};

QwtAxesMask::QwtAxesMask()
{
    d_data = new PrivateData();
}

QwtAxesMask::~QwtAxesMask()
{
    delete d_data;
}

/*!
   \brief En/Disable an axis

   Only Axes that are enabled will be zoomed.
   All other axes will remain unchanged.

   \param axis Axis, see QwtPlot::Axis
   \param on On/Off

   \sa isAxisEnabled()
*/
void QwtAxesMask::setEnabled( int axisPos, int id, bool on )
{
    if ( axisPos < 0 || axisPos >= QwtPlot::NumAxisPositions )
        return;

    QList<int> &axes = d_data->disabledAxes[ axisPos ];
    
    QList<int>::iterator it = qLowerBound( axes.begin(), axes.end(), id );

    const bool isDisabled = ( it != axes.end() ) && ( *it != id );

    if ( on )
    {
        if ( isDisabled )
            axes.erase( it );
    }
    else
    {
        if ( !isDisabled )
            axes.insert( it, id );
    }
}

/*!
   Test if an axis is enabled

   \param axisPos Axis position, see QwtPlot::Axis
   \return True, if the axis is enabled

   \sa setAxisEnabled()
*/
bool QwtAxesMask::isEnabled( int axisPos, int id ) const
{
    if ( axisPos >= 0 && axisPos < QwtPlot::NumAxisPositions )
    {
        const QList<int> &axes = d_data->disabledAxes[ axisPos ];
        return qFind( axes, id ) != axes.end();
    }

    return true;
}
