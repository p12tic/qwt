/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 1997   Josef Wilgen
 * Copyright (C) 2002   Uwe Rathmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#include "qwt_plot.h"
#include "qwt_math.h"
#include "qwt_scale_widget.h"
#include "qwt_scale_div.h"
#include "qwt_scale_engine.h"

class QwtPlotAxisData
{
public:
    QwtPlotAxisData():
        isVisible( true ),
        doAutoScale( true ),
        minValue( 0.0 ),
        maxValue( 1000.0 ),
        stepSize( 0.0 ),
        maxMajor( 8 ),
        maxMinor( 5 ),
        isValid( false ),
        scaleEngine( new QwtLinearScaleEngine() ),
        scaleWidget( NULL )
    {
    }

    void initWidget( QwtScaleDraw::Alignment align, const QString& name, QwtPlot* plot )
    {
        scaleWidget = new QwtScaleWidget( align, plot );
        scaleWidget->setObjectName( name ); 

#if 1
        // better find the font sizes from the application font
        const QFont fscl( plot->fontInfo().family(), 10 );
        const QFont fttl( plot->fontInfo().family(), 12, QFont::Bold );
#endif
    
        scaleWidget->setTransformation( scaleEngine->transformation() );

        scaleWidget->setFont( fscl );
        scaleWidget->setMargin( 2 );

        QwtText text = scaleWidget->title();
        text.setFont( fttl );
        scaleWidget->setTitle( text );
    }

    bool isVisible;
    bool doAutoScale;

    double minValue;
    double maxValue;
    double stepSize;

    int maxMajor;
    int maxMinor;

    bool isValid;

    QwtScaleDiv scaleDiv;
    QwtScaleEngine *scaleEngine;
    QwtScaleWidget *scaleWidget;
};

class QwtPlot::ScaleData
{
public:
    ScaleData()
    {
        for ( int axisPos = 0; axisPos < QwtPlot::NumAxisPositions; axisPos++ )
            d[axisPos].axisData.resize( 1 );
    }

    ~ScaleData()
    {
        for ( int axisPos = 0; axisPos < NumAxisPositions; axisPos++ )
        {
            for ( int i = 0; i < d[axisPos].axisData.size(); i++ )
            {
                delete d[axisPos].axisData[i].scaleEngine;
            }
        }
    }

    inline int axesCount( int pos ) const
    {
        if ( pos < 0 || pos >= QwtPlot::NumAxisPositions )
            return -1;

        return d[pos].axisData.count();
    }

    inline QwtPlotAxisData &axisData( int pos, int id = 0 )
    {
        return d[pos].axisData[id];
    }

    inline const QwtPlotAxisData &axisData( int pos, int id = 0 ) const
    {
        return d[pos].axisData[id];
    }

    struct 
    {
        QVector< QwtPlotAxisData > axisData;

    } d[ QwtPlot::NumAxisPositions ];
};

//! Initialize axes
void QwtPlot::initScaleData()
{
    d_scaleData = new ScaleData();

    d_scaleData->axisData( yLeft, 0 ).initWidget( QwtScaleDraw::LeftScale, "QwtPlotAxisYLeft", this );
    d_scaleData->axisData( yRight, 0 ).initWidget( QwtScaleDraw::RightScale, "QwtPlotAxisYRight", this );
    d_scaleData->axisData( xTop, 0 ).initWidget( QwtScaleDraw::TopScale, "QwtPlotAxisXTop", this );
    d_scaleData->axisData( xBottom, 0 ).initWidget( QwtScaleDraw::BottomScale, "QwtPlotAxisXBottom", this );

    d_scaleData->axisData( yRight, 0 ).isVisible = false;
    d_scaleData->axisData( xTop, 0 ).isVisible = false;
}

void QwtPlot::deleteScaleData()
{
    delete d_scaleData;
    d_scaleData = NULL;
}

int QwtPlot::axesCount( int axisPos ) const
{
    return d_scaleData->axesCount( axisPos );
}

/*!
  \return \c true if the specified axis exists, otherwise \c false
  \param axisPos axis index
 */
bool QwtPlot::isAxisValid( int axisPos, int id ) const
{
    return d_scaleData->axesCount( axisPos ) > id;
}

bool QwtPlot::hasVisibleAxes( int axisPos ) const
{
    if ( axisPos < 0 || axisPos >= QwtPlot::NumAxisPositions )
        return false;

    const int axesCount = d_scaleData->axesCount( axisPos );
    for ( int i = 0; i < axesCount; i++ )
    {
        if ( d_scaleData->axisData( axisPos, i ).isVisible )
            return true;
    }

    return false;
}

/*!
  \return Scale widget of the specified axis, or NULL if axisPos is invalid.
  \param axisPos Axis position
*/
const QwtScaleWidget *QwtPlot::axisWidget( int axisPos, int id  ) const
{
    if ( isAxisValid( axisPos, id ) )
        return d_scaleData->axisData( axisPos, id ).scaleWidget;

    return NULL;
}

/*!
  \return Scale widget of the specified axis, or NULL if axisPos is invalid.
  \param axisPos Axis position
*/
QwtScaleWidget *QwtPlot::axisWidget( int axisPos, int id )
{
    if ( isAxisValid( axisPos, id ) )
        return d_scaleData->axisData(axisPos, id).scaleWidget;

    return NULL;
}

/*!
  Change the scale engine for an axis

  \param axisPos Axis position
  \param scaleEngine Scale engine

  \sa axisScaleEngine()
*/
void QwtPlot::setAxisScaleEngine( int axisPos, int id, QwtScaleEngine *scaleEngine )
{
    if ( isAxisValid( axisPos, id ) && scaleEngine != NULL )
    {
        QwtPlotAxisData &d = d_scaleData->axisData(axisPos, id);

        delete d.scaleEngine;
        d.scaleEngine = scaleEngine;

        d.scaleWidget->setTransformation( scaleEngine->transformation() );

        d.isValid = false;

        autoRefresh();
    }
}

/*!
  \param axisPos Axis position
  \return Scale engine for a specific axis
*/
QwtScaleEngine *QwtPlot::axisScaleEngine( int axisPos, int id )
{
    if ( isAxisValid( axisPos, id ) )
        return d_scaleData->axisData( axisPos, id ).scaleEngine;
    else
        return NULL;
}

/*!
  \param axisPos Axis position
  \return Scale engine for a specific axis
*/
const QwtScaleEngine *QwtPlot::axisScaleEngine( int axisPos, int id ) const
{
    if ( isAxisValid( axisPos, id ) )
        return d_scaleData->axisData( axisPos, id ).scaleEngine;
    else
        return NULL;
}
/*!
  \return \c True, if autoscaling is enabled
  \param axisPos Axis position
*/
bool QwtPlot::axisAutoScale( int axisPos, int id ) const
{
    if ( isAxisValid( axisPos, id ) )
        return d_scaleData->axisData( axisPos, id ).doAutoScale;
    else
        return false;
}

/*!
  \return \c True, if a specified axis is visible
  \param axisPos Axis position
*/
bool QwtPlot::isAxisVisible( int axisPos, int id ) const
{
    if ( isAxisValid( axisPos, id ) )
        return d_scaleData->axisData( axisPos, id ).isVisible;
    else
        return false;
}

/*!
  \return The font of the scale labels for a specified axis
  \param axisPos Axis position
*/
QFont QwtPlot::axisFont( int axisPos, int id ) const
{
    if ( isAxisValid( axisPos, id ) )
        return axisWidget( axisPos, id )->font();
    else
        return QFont();

}

/*!
  \return The maximum number of major ticks for a specified axis
  \param axisPos Axis position
  \sa setAxisMaxMajor(), QwtScaleEngine::divideScale()
*/
int QwtPlot::axisMaxMajor( int axisPos, int id ) const
{
    if ( isAxisValid( axisPos, id ) )
        return d_scaleData->axisData(axisPos, id).maxMajor;
    else
        return 0;
}

/*!
  \return the maximum number of minor ticks for a specified axis
  \param axisPos Axis position
  \sa setAxisMaxMinor(), QwtScaleEngine::divideScale()
*/
int QwtPlot::axisMaxMinor( int axisPos, int id ) const
{
    if ( isAxisValid( axisPos, id ) )
        return d_scaleData->axisData( axisPos, id ).maxMinor;
    else
        return 0;
}

/*!
  \brief Return the scale division of a specified axis

  axisScaleDiv(axisPos).lowerBound(), axisScaleDiv(axisPos).upperBound()
  are the current limits of the axis scale.

  \param axisPos Axis position
  \return Scale division

  \sa QwtScaleDiv, setAxisScaleDiv(), QwtScaleEngine::divideScale()
*/
const QwtScaleDiv &QwtPlot::axisScaleDiv( int axisPos, int id ) const
{
    return d_scaleData->axisData( axisPos, id ).scaleDiv;
}

/*!
  \brief Return the scale draw of a specified axis

  \param axisPos Axis position
  \return Specified scaleDraw for axis, or NULL if axis is invalid.
*/
const QwtScaleDraw *QwtPlot::axisScaleDraw( int axisPos, int id ) const
{
    if ( !isAxisValid( axisPos, id ) )
        return NULL;

    return axisWidget( axisPos, id )->scaleDraw();
}

/*!
  \brief Return the scale draw of a specified axis

  \param axisPos Axis position
  \return Specified scaleDraw for axis, or NULL if axis is invalid.
*/
QwtScaleDraw *QwtPlot::axisScaleDraw( int axisPos, int id )
{
    if ( !isAxisValid( axisPos, id ) )
        return NULL;

    return axisWidget( axisPos, id )->scaleDraw();
}

/*!
  \brief Return the step size parameter that has been set in setAxisScale. 

  This doesn't need to be the step size of the current scale.

  \param axisPos Axis position
  \return step size parameter value

   \sa setAxisScale(), QwtScaleEngine::divideScale()
*/
double QwtPlot::axisStepSize( int axisPos, int id ) const
{
    if ( !isAxisValid( axisPos, id ) )
        return 0;

    return d_scaleData->axisData( axisPos, id ).stepSize;
}

/*!
  \brief Return the current interval of the specified axis

  This is only a convenience function for axisScaleDiv( axisPos )->interval();
  
  \param axisPos Axis position
  \return Scale interval

  \sa QwtScaleDiv, axisScaleDiv()
*/
QwtInterval QwtPlot::axisInterval( int axisPos, int id ) const
{
    if ( !isAxisValid( axisPos, id ) )
        return QwtInterval();

    return d_scaleData->axisData( axisPos, id ).scaleDiv.interval();
}

/*!
  \return Title of a specified axis
  \param axisPos Axis position
*/
QwtText QwtPlot::axisTitle( int axisPos, int id ) const
{
    if ( isAxisValid( axisPos, id ) )
        return axisWidget( axisPos, id )->title();
    else
        return QwtText();
}

/*!
  \brief Enable or disable a specified axis

  Even when an axis is not visible curves, markers and can be attached
  to it, and transformation of screen coordinates into values works as normal.

  Only xBottom and yLeft are visible by default.

  \param axisPos Axis position
  \param on \c true (visisble) or \c false (hidden)
*/
void QwtPlot::setAxisVisible( int axisPos, int id, bool on )
{
    if ( isAxisValid( axisPos, id ) && on != d_scaleData->axisData( axisPos, id ).isVisible )
    {
        d_scaleData->axisData( axisPos, id ).isVisible = on;
        updateLayout();
    }
}

/*!
  Transform the x or y coordinate of a position in the
  drawing region into a value.

  \param axisPos Axis position
  \param pos position

  \return Position as axis coordinate

  \warning The position can be an x or a y coordinate,
           depending on the specified axis.
*/
double QwtPlot::invTransform( int axisPos, int id, int pos ) const
{
    if ( isAxisValid( axisPos, id ) )
        return( canvasMap( axisPos, id ).invTransform( pos ) );
    else
        return 0.0;
}


/*!
  \brief Transform a value into a coordinate in the plotting region

  \param axisPos Axis position
  \param value value
  \return X or Y coordinate in the plotting region corresponding
          to the value.
*/
double QwtPlot::transform( int axisPos, int id, double value ) const
{
    if ( isAxisValid( axisPos, id ) )
        return( canvasMap( axisPos, id ).transform( value ) );
    else
        return 0.0;
}

/*!
  \brief Change the font of an axis

  \param axisPos Axis position
  \param font Font
  \warning This function changes the font of the tick labels,
           not of the axis title.
*/
void QwtPlot::setAxisFont( int axisPos, int id, const QFont &font )
{
    if ( isAxisValid( axisPos, id ) )
        axisWidget( axisPos, id )->setFont( font );
}

/*!
  \brief Enable autoscaling for a specified axis

  This member function is used to switch back to autoscaling mode
  after a fixed scale has been set. Autoscaling is enabled by default.

  \param axisPos Axis position
  \param on On/Off
  \sa setAxisScale(), setAxisScaleDiv(), updateAxes()

  \note The autoscaling flag has no effect until updateAxes() is executed
        ( called by replot() ).
*/
void QwtPlot::setAxisAutoScale( int axisPos, int id, bool on )
{
    if ( isAxisValid( axisPos, id ) && ( d_scaleData->axisData( axisPos, id ).doAutoScale != on ) )
    {
        d_scaleData->axisData( axisPos, id ).doAutoScale = on;
        autoRefresh();
    }
}

/*!
  \brief Disable autoscaling and specify a fixed scale for a selected axis.

  In updateAxes() the scale engine calculates a scale division from the 
  specified parameters, that will be assigned to the scale widget. So 
  updates of the scale widget usually happen delayed with the next replot.

  \param axisPos Axis position
  \param min Minimum of the scale
  \param max Maximum of the scale
  \param stepSize Major step size. If <code>step == 0</code>, the step size is
                  calculated automatically using the maxMajor setting.

  \sa setAxisMaxMajor(), setAxisAutoScale(), axisStepSize(), QwtScaleEngine::divideScale()
*/
void QwtPlot::setAxisScaleDiv( int axisPos, int id, double min, double max, double stepSize )
{
    if ( isAxisValid( axisPos, id ) )
    {
        QwtPlotAxisData &d = d_scaleData->axisData(axisPos, id);

        d.doAutoScale = false;
        d.isValid = false;

        d.minValue = min;
        d.maxValue = max;
        d.stepSize = stepSize;

        autoRefresh();
    }
}

/*!
  \brief Disable autoscaling and specify a fixed scale for a selected axis.

  The scale division will be stored locally only until the next call
  of updateAxes(). So updates of the scale widget usually happen delayed with 
  the next replot.

  \param axisPos Axis position
  \param scaleDiv Scale division

  \sa setAxisScale(), setAxisAutoScale()
*/
void QwtPlot::setAxisScaleDiv( int axisPos, int id, const QwtScaleDiv &scaleDiv )
{
    if ( isAxisValid( axisPos, id ) )
    {
        QwtPlotAxisData &d = d_scaleData->axisData(axisPos, id);

        d.doAutoScale = false;
        d.scaleDiv = scaleDiv;
        d.isValid = true;

        autoRefresh();
    }
}

/*!
  \brief Set a scale draw

  \param axisPos Axis position
  \param scaleDraw Object responsible for drawing scales.

  By passing scaleDraw it is possible to extend QwtScaleDraw
  functionality and let it take place in QwtPlot. Please note
  that scaleDraw has to be created with new and will be deleted
  by the corresponding QwtScale member ( like a child object ).

  \sa QwtScaleDraw, QwtScaleWidget
  \warning The attributes of scaleDraw will be overwritten by those of the
           previous QwtScaleDraw.
*/

void QwtPlot::setAxisScaleDraw( int axisPos, int id, QwtScaleDraw *scaleDraw )
{
    if ( isAxisValid( axisPos, id ) )
    {
        axisWidget( axisPos, id )->setScaleDraw( scaleDraw );
        autoRefresh();
    }
}

/*!
  Change the alignment of the tick labels

  \param axisPos Axis position
  \param alignment Or'd Qt::AlignmentFlags see <qnamespace.h>

  \sa QwtScaleDraw::setLabelAlignment()
*/
void QwtPlot::setAxisLabelAlignment( int axisPos, int id, Qt::Alignment alignment )
{
    if ( isAxisValid( axisPos, id ) )
        axisWidget( axisPos, id )->setLabelAlignment( alignment );
}

/*!
  Rotate all tick labels

  \param axisPos Axis position
  \param rotation Angle in degrees. When changing the label rotation,
                  the label alignment might be adjusted too.

  \sa QwtScaleDraw::setLabelRotation(), setAxisLabelAlignment()
*/
void QwtPlot::setAxisLabelRotation( int axisPos, int id, double rotation )
{
    if ( isAxisValid( axisPos, id ) )
        axisWidget( axisPos, id )->setLabelRotation( rotation );
}

/*!
  Set the maximum number of minor scale intervals for a specified axis

  \param axisPos Axis position
  \param maxMinor Maximum number of minor steps

  \sa axisMaxMinor()
*/
void QwtPlot::setAxisMaxMinor( int axisPos, int id, int maxMinor )
{
    if ( isAxisValid( axisPos, id ) )
    {
        maxMinor = qBound( 0, maxMinor, 100 );

        QwtPlotAxisData &d = d_scaleData->axisData(axisPos, id);
        if ( maxMinor != d.maxMinor )
        {
            d.maxMinor = maxMinor;
            d.isValid = false;
            autoRefresh();
        }
    }
}

/*!
  Set the maximum number of major scale intervals for a specified axis

  \param axisPos Axis position
  \param maxMajor Maximum number of major steps

  \sa axisMaxMajor()
*/
void QwtPlot::setAxisMaxMajor( int axisPos, int id, int maxMajor )
{
    if ( isAxisValid( axisPos, id ) )
    {
        maxMajor = qBound( 1, maxMajor, 10000 );

        QwtPlotAxisData &d = d_scaleData->axisData(axisPos, id);
        if ( maxMajor != d.maxMajor )
        {
            d.maxMajor = maxMajor;
            d.isValid = false;
            autoRefresh();
        }
    }
}

/*!
  \brief Change the title of a specified axis

  \param axisPos Axis position
  \param title axis title
*/
void QwtPlot::setAxisTitle( int axisPos, int id, const QString &title )
{
    if ( isAxisValid( axisPos, id ) )
        axisWidget( axisPos, id )->setTitle( title );
}

/*!
  \brief Change the title of a specified axis

  \param axisPos Axis position
  \param title Axis title
*/
void QwtPlot::setAxisTitle( int axisPos, int id, const QwtText &title )
{
    if ( isAxisValid( axisPos, id ) )
        axisWidget( axisPos, id )->setTitle( title );
}

/*! 
  \brief Rebuild the axes scales

  In case of autoscaling the boundaries of a scale are calculated 
  from the bounding rectangles of all plot items, having the 
  QwtPlotItem::AutoScale flag enabled ( QwtScaleEngine::autoScale() ). 
  Then a scale division is calculated ( QwtScaleEngine::didvideScale() ) 
  and assigned to scale widget.

  When the scale boundaries have been assigned with setAxisScale() a 
  scale division is calculated ( QwtScaleEngine::didvideScale() )
  for this interval and assigned to the scale widget.

  When the scale has been set explicitly by setAxisScaleDiv() the 
  locally stored scale division gets assigned to the scale widget.

  The scale widget indicates modifications by emitting a 
  QwtScaleWidget::scaleDivChanged() signal.

  updateAxes() is usually called by replot(). 

  \sa setAxisAutoScale(), setAxisScale(), setAxisScaleDiv(), replot()
      QwtPlotItem::boundingRect()
 */
void QwtPlot::updateAxes()
{
    // Find bounding interval of the item data
    // for all axes, where autoscaling is enabled

    QwtInterval intv[NumAxisPositions];

    const QwtPlotItemList& itmList = itemList();

    QwtPlotItemIterator it;
    for ( it = itmList.begin(); it != itmList.end(); ++it )
    {
        const QwtPlotItem *item = *it;

        if ( !item->testItemAttribute( QwtPlotItem::AutoScale ) )
            continue;

        if ( !item->isVisible() )
            continue;

        if ( axisAutoScale( item->xAxisPos(), item->xAxisId() ) || 
            axisAutoScale( item->yAxisPos(), item->yAxisId() ) )
        {
            const QRectF rect = item->boundingRect();

            if ( rect.width() >= 0.0 )
                intv[item->xAxisPos()] |= QwtInterval( rect.left(), rect.right() );

            if ( rect.height() >= 0.0 )
                intv[item->yAxisPos()] |= QwtInterval( rect.top(), rect.bottom() );
        }
    }

    // Adjust scales

    for ( int axisPos = 0; axisPos < NumAxisPositions; axisPos++ )
    {
        QwtPlotAxisData &d = d_scaleData->axisData(axisPos);

        double minValue = d.minValue;
        double maxValue = d.maxValue;
        double stepSize = d.stepSize;

        if ( d.doAutoScale && intv[axisPos].isValid() )
        {
            d.isValid = false;

            minValue = intv[axisPos].minValue();
            maxValue = intv[axisPos].maxValue();

            d.scaleEngine->autoScale( d.maxMajor,
                minValue, maxValue, stepSize );
        }
        if ( !d.isValid )
        {
            d.scaleDiv = d.scaleEngine->divideScale(
                minValue, maxValue,
                d.maxMajor, d.maxMinor, stepSize );
            d.isValid = true;
        }

        QwtScaleWidget *scaleWidget = axisWidget( axisPos, QWT_DUMMY_ID );
        scaleWidget->setScaleDiv( d.scaleDiv );

        int startDist, endDist;
        scaleWidget->getBorderDistHint( startDist, endDist );
        scaleWidget->setBorderDist( startDist, endDist );
    }

    for ( it = itmList.begin(); it != itmList.end(); ++it )
    {
        QwtPlotItem *item = *it;
        if ( item->testItemInterest( QwtPlotItem::ScaleInterest ) )
        {
            item->updateScaleDiv( axisScaleDiv( item->xAxisPos(), item->xAxisId() ),
                axisScaleDiv( item->yAxisPos(), item->yAxisId() ) );
        }
    }
}

