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
void QwtAxesMask::setEnabled( QwtAxisId axisId, bool on )
{
    if ( axisId.pos < 0 || axisId.pos >= QwtPlot::NumAxisPositions )
        return;

    QList<int> &axes = d_data->disabledAxes[ axisId.pos ];
    
    QList<int>::iterator it = qLowerBound( axes.begin(), axes.end(), axisId.id );

    const bool isDisabled = ( it != axes.end() ) && ( *it != axisId.id );

    if ( on )
    {
        if ( isDisabled )
            axes.erase( it );
    }
    else
    {
        if ( !isDisabled )
            axes.insert( it, axisId.id );
    }
}

/*!
   Test if an axis is enabled

   \param axisPos Axis position, see QwtPlot::Axis
   \return True, if the axis is enabled

   \sa setAxisEnabled()
*/
bool QwtAxesMask::isEnabled( QwtAxisId axisId ) const
{
    if ( axisId.pos >= 0 && axisId.pos < QwtPlot::NumAxisPositions )
    {
        const QList<int> &axes = d_data->disabledAxes[ axisId.pos ];
        return qFind( axes, axisId.id ) != axes.end();
    }

    return true;
}
