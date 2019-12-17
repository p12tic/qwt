/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 1997   Josef Wilgen
 * Copyright (C) 2002   Uwe Rathmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#include "qwt_series_data.h"
#include "qwt_point_polar.h"

class QwtSeriesDataBase::PrivateData
{
public:
    PrivateData() :
        cachedBoundingRect( 0.0, 0.0, -1.0, -1.0 ),
        seriesFlags(0)
    {}

    mutable QRectF cachedBoundingRect;
    SeriesFlags seriesFlags;
};

QwtSeriesDataBase::QwtSeriesDataBase()
{
    d_data = new PrivateData();
}

QwtSeriesDataBase::~QwtSeriesDataBase()
{
    delete d_data;
}

void QwtSeriesDataBase::setCachedBoundingRect( const QRectF& rect ) const
{
    d_data->cachedBoundingRect = rect;
}

void QwtSeriesDataBase::clearCachedBoundingRect() const
{
    d_data->cachedBoundingRect = QRect( 0.0, 0.0, -1.0, -1.0 );
}

QRectF QwtSeriesDataBase::cachedBoundingRect() const
{
    return d_data->cachedBoundingRect;
}

void QwtSeriesDataBase::setSeriesFlags( SeriesFlags flags )
{
    d_data->seriesFlags = flags;
}

QwtSeriesDataBase::SeriesFlags QwtSeriesDataBase::seriesFlags() const
{
    return d_data->seriesFlags;
}

QRectF qwtBoundingRect( const QPointF &sample )
{
    return QRectF( sample.x(), sample.y(), 0.0, 0.0 );
}

QRectF qwtBoundingRect( const QwtPoint3D &sample )
{
    return QRectF( sample.x(), sample.y(), 0.0, 0.0 );
}

QRectF qwtBoundingRect( const QwtPointPolar &sample )
{
    return QRectF( sample.azimuth(), sample.radius(), 0.0, 0.0 );
}

QRectF qwtBoundingRect( const QwtIntervalSample &sample )
{
    return QRectF( sample.interval.minValue(), sample.value,
        sample.interval.maxValue() - sample.interval.minValue(), 0.0 );
}

QRectF qwtBoundingRect( const QwtSetSample &sample )
{
    if ( sample.set.empty() )
        return QRectF( sample.value, 0.0, 0.0, -1.0 );

    double minY = sample.set[0];
    double maxY = sample.set[0];

    for ( int i = 1; i < sample.set.size(); i++ )
    {
        if ( sample.set[i] < minY )
            minY = sample.set[i];

        if ( sample.set[i] > maxY )
            maxY = sample.set[i];
    }

    return QRectF( sample.value, minY, 0.0, maxY - minY );
}

QRectF qwtBoundingRect( const QwtOHLCSample &sample )
{
    const QwtInterval interval = sample.boundingInterval();
    return QRectF( interval.minValue(), sample.time, interval.width(), 0.0 );
}

static inline QRectF qwtMergeBoundingRectNoCheck( const QRectF& rect1, const QRectF& rect2 )
{
    double top = qMin( rect1.top(), rect2.top() );
    double bottom = qMax( rect1.bottom(), rect2.bottom() );
    double left = qMin( rect1.left(), rect2.left() );
    double right = qMax( rect1.right(), rect2.right() );

    return QRectF( left, top, right - left, bottom - top );
}

/*!
   \brief Merges two bounding rectangles

    Differently from QRectF::united or QRectF::operator|, rectangles of zero size are
    accepted. If either width or height of the rectangle is negative, the other rectangle
    is returned.

    \param rect1 Rectangle to merge
    \param rect2 Rectangle to merge
    \return Merged rectangle
 */
QRectF qwtMergeBoundingRect( const QRectF& rect1, const QRectF& rect2 )
{
    if ( rect1.width() < 0 || rect1.height() < 0 )
        return rect2;
    if ( rect2.width() < 0 || rect2.height() < 0 )
        return rect1;

    return qwtMergeBoundingRectNoCheck( rect1, rect2 );
}


/*!
  \brief Calculate the bounding rectangle of a series subset

  Slow implementation, that iterates over the series.

  \param series Series
  \param from Index of the first sample, <= 0 means from the beginning
  \param to Index of the last sample, < 0 means to the end

  \return Bounding rectangle
*/

template <class T>
QRectF qwtBoundingRectT(
    const QwtSeriesData<T>& series, int from, int to )
{
    QRectF boundingRect( 1.0, 1.0, -2.0, -2.0 ); // invalid;

    if ( from < 0 )
        from = 0;

    if ( to < 0 )
        to = series.size() - 1;

    if ( to < from )
        return boundingRect;

    const bool processNaN = series.seriesFlags() & QwtSeriesDataBase::MayContainNaNs;

    int i;
    for ( i = from; i <= to; i++ )
    {
        const T& sample = series.sample( i );
        if ( processNaN && qwtIsNaN( sample ) )
            continue;

        const QRectF rect = qwtBoundingRect( sample );
        if ( rect.width() >= 0.0 && rect.height() >= 0.0 )
        {
            boundingRect = rect;
            i++;
            break;
        }
    }

    for ( ; i <= to; i++ )
    {
        const T& sample = series.sample( i );
        if ( processNaN && qwtIsNaN( sample ) )
            continue;

        const QRectF rect = qwtBoundingRect( sample );
        if ( rect.width() >= 0.0 && rect.height() >= 0.0 )
        {
            boundingRect = qwtMergeBoundingRectNoCheck( boundingRect, rect );
        }
    }

    return boundingRect;
}

/*!
  \brief Calculate the bounding rectangle of a series subset

  Slow implementation, that iterates over the series.

  \param series Series
  \param from Index of the first sample, <= 0 means from the beginning
  \param to Index of the last sample, < 0 means to the end

  \return Bounding rectangle
*/
QRectF qwtBoundingRect(
    const QwtSeriesData<QPointF> &series, int from, int to )
{
    return qwtBoundingRectT<QPointF>( series, from, to );
}

/*!
  \brief Calculate the bounding rectangle of a series subset

  Slow implementation, that iterates over the series.

  \param series Series
  \param from Index of the first sample, <= 0 means from the beginning
  \param to Index of the last sample, < 0 means to the end

  \return Bounding rectangle
*/
QRectF qwtBoundingRect(
    const QwtSeriesData<QwtPoint3D> &series, int from, int to )
{
    return qwtBoundingRectT<QwtPoint3D>( series, from, to );
}

/*!
  \brief Calculate the bounding rectangle of a series subset

  The horizontal coordinates represent the azimuth, the
  vertical coordinates the radius.

  Slow implementation, that iterates over the series.

  \param series Series
  \param from Index of the first sample, <= 0 means from the beginning
  \param to Index of the last sample, < 0 means to the end

  \return Bounding rectangle
*/
QRectF qwtBoundingRect(
    const QwtSeriesData<QwtPointPolar> &series, int from, int to )
{
    return qwtBoundingRectT<QwtPointPolar>( series, from, to );
}

/*!
  \brief Calculate the bounding rectangle of a series subset

  Slow implementation, that iterates over the series.

  \param series Series
  \param from Index of the first sample, <= 0 means from the beginning
  \param to Index of the last sample, < 0 means to the end

  \return Bounding rectangle
*/
QRectF qwtBoundingRect(
    const QwtSeriesData<QwtIntervalSample>& series, int from, int to )
{
    return qwtBoundingRectT<QwtIntervalSample>( series, from, to );
}

/*!
  \brief Calculate the bounding rectangle of a series subset

  Slow implementation, that iterates over the series.

  \param series Series
  \param from Index of the first sample, <= 0 means from the beginning
  \param to Index of the last sample, < 0 means to the end

  \return Bounding rectangle
*/
QRectF qwtBoundingRect(
    const QwtSeriesData<QwtOHLCSample>& series, int from, int to )
{
    return qwtBoundingRectT<QwtOHLCSample>( series, from, to );
}

/*!
  \brief Calculate the bounding rectangle of a series subset

  Slow implementation, that iterates over the series.

  \param series Series
  \param from Index of the first sample, <= 0 means from the beginning
  \param to Index of the last sample, < 0 means to the end

  \return Bounding rectangle
*/
QRectF qwtBoundingRect(
    const QwtSeriesData<QwtSetSample>& series, int from, int to )
{
    return qwtBoundingRectT<QwtSetSample>( series, from, to );
}

/*!
   Constructor
   \param samples Samples
*/
QwtPointSeriesData::QwtPointSeriesData(
        const QVector<QPointF> &samples ):
    QwtArraySeriesData<QPointF>( samples )
{
}

/*!
  \brief Calculate the bounding rectangle

  The bounding rectangle is calculated once by iterating over all
  points and is stored for all following requests.

  \return Bounding rectangle
*/
QRectF QwtPointSeriesData::boundingRect() const
{
    QRectF cachedRect = cachedBoundingRect();
    if ( cachedRect.width() < 0.0 )
    {
        cachedRect = qwtBoundingRect( *this );
        setCachedBoundingRect( cachedRect );
    }

    return cachedRect;
}

/*!
   Constructor
   \param samples Samples
*/
QwtPoint3DSeriesData::QwtPoint3DSeriesData(
        const QVector<QwtPoint3D> &samples ):
    QwtArraySeriesData<QwtPoint3D>( samples )
{
}

/*!
  \brief Calculate the bounding rectangle

  The bounding rectangle is calculated once by iterating over all
  points and is stored for all following requests.

  \return Bounding rectangle
*/
QRectF QwtPoint3DSeriesData::boundingRect() const
{
    QRectF cachedRect = cachedBoundingRect();
    if ( cachedRect.width() < 0.0 )
    {
        cachedRect = qwtBoundingRect( *this );
        setCachedBoundingRect( cachedRect );
    }

    return cachedRect;
}

/*!
   Constructor
   \param samples Samples
*/
QwtIntervalSeriesData::QwtIntervalSeriesData(
        const QVector<QwtIntervalSample> &samples ):
    QwtArraySeriesData<QwtIntervalSample>( samples )
{
}

/*!
  \brief Calculate the bounding rectangle

  The bounding rectangle is calculated once by iterating over all
  points and is stored for all following requests.

  \return Bounding rectangle
*/
QRectF QwtIntervalSeriesData::boundingRect() const
{
    QRectF cachedRect = cachedBoundingRect();
    if ( cachedRect.width() < 0.0 )
    {
        cachedRect = qwtBoundingRect( *this );
        setCachedBoundingRect( cachedRect );
    }

    return cachedRect;
}

/*!
   Constructor
   \param samples Samples
*/
QwtSetSeriesData::QwtSetSeriesData(
        const QVector<QwtSetSample> &samples ):
    QwtArraySeriesData<QwtSetSample>( samples )
{
}

/*!
  \brief Calculate the bounding rectangle

  The bounding rectangle is calculated once by iterating over all
  points and is stored for all following requests.

  \return Bounding rectangle
*/
QRectF QwtSetSeriesData::boundingRect() const
{
    QRectF cachedRect = cachedBoundingRect();
    if ( cachedRect.width() < 0.0 )
    {
        cachedRect = qwtBoundingRect( *this );
        setCachedBoundingRect( cachedRect );
    }

    return cachedRect;
}

/*!
   Constructor
   \param samples Samples
*/
QwtTradingChartData::QwtTradingChartData(
        const QVector<QwtOHLCSample> &samples ):
    QwtArraySeriesData<QwtOHLCSample>( samples )
{
}

/*!
  \brief Calculate the bounding rectangle

  The bounding rectangle is calculated once by iterating over all
  points and is stored for all following requests.

  \return Bounding rectangle
*/
QRectF QwtTradingChartData::boundingRect() const
{
    QRectF cachedRect = cachedBoundingRect();
    if ( cachedRect.width() < 0.0 )
    {
        cachedRect = qwtBoundingRect( *this );
        setCachedBoundingRect( cachedRect );
    }

    return cachedRect;
}
