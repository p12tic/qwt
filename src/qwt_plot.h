/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 1997   Josef Wilgen
 * Copyright (C) 2002   Uwe Rathmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#ifndef QWT_PLOT_H
#define QWT_PLOT_H

#include "qwt_global.h"
#include "qwt_text.h"
#include "qwt_plot_dict.h"
#include "qwt_scale_map.h"
#include "qwt_interval.h"
#include <qframe.h>
#include <qlist.h>
#include <qvariant.h>

class QwtPlotLayout;
class QwtAbstractLegend;
class QwtScaleWidget;
class QwtScaleEngine;
class QwtScaleDiv;
class QwtScaleDraw;
class QwtTextLabel;

#define QWT_COMPAT 1 // flag to disable compatibilities - will be removed later
#define QWT_DUMMY_ID 0 // dummy id to help for migrating the code - will be removed later

/*!
  \brief A 2-D plotting widget

  QwtPlot is a widget for plotting two-dimensional graphs.
  An unlimited number of plot items can be displayed on
  its canvas. Plot items might be curves (QwtPlotCurve), markers
  (QwtPlotMarker), the grid (QwtPlotGrid), or anything else derived
  from QwtPlotItem.
  A plot can have up to four axes, with each plot item attached to an x- and
  a y axis. The scales at the axes can be explicitly set (QwtScaleDiv), or
  are calculated from the plot items, using algorithms (QwtScaleEngine) which
  can be configured separately for each axis.

  The simpleplot example is a good starting point to see how to set up a 
  plot widget.

  \image html plot.png

  \par Example
  The following example shows (schematically) the most simple
  way to use QwtPlot. By default, only the left and bottom axes are
  visible and their scales are computed automatically.
  \verbatim
#include <qwt_plot.h>
#include <qwt_plot_curve.h>

QwtPlot *myPlot = new QwtPlot("Two Curves", parent);

// add curves
QwtPlotCurve *curve1 = new QwtPlotCurve("Curve 1");
QwtPlotCurve *curve2 = new QwtPlotCurve("Curve 2");

// connect or copy the data to the curves
curve1->setData(...);
curve2->setData(...);

curve1->attach(myPlot);
curve2->attach(myPlot);

// finally, refresh the plot
myPlot->replot();
\endverbatim
*/

class QWT_EXPORT QwtPlot: public QFrame, public QwtPlotDict
{
    Q_OBJECT

    Q_PROPERTY( QBrush canvasBackground 
        READ canvasBackground WRITE setCanvasBackground )
    Q_PROPERTY( bool autoReplot READ autoReplot WRITE setAutoReplot )

#if 0
    // This property is intended to configure the plot
    // widget from a special dialog in the deigner plugin.
    // Disabled until such a dialog has been implemented.

    Q_PROPERTY( QString propertiesDocument
        READ grabProperties WRITE applyProperties )
#endif

public:
    //! \brief Axis position
    enum AxisPosition
    {
        //! Y axis left of the canvas
        yLeft,

        //! Y axis right of the canvas
        yRight,

        //! X axis below the canvas
        xBottom,

        //! X axis above the canvas
        xTop,

        //! Number of axes
        NumAxisPositions
    };

    /*!
        Position of the legend, relative to the canvas.

        \sa insertLegend()
     */
    enum LegendPosition
    {
        //! The legend will be left from the QwtPlot::yLeft axis.
        LeftLegend,

        //! The legend will be right from the QwtPlot::yRight axis.
        RightLegend,

        //! The legend will be below the footer 
        BottomLegend,

        //! The legend will be above the title
        TopLegend
    };

    explicit QwtPlot( QWidget * = NULL );
    explicit QwtPlot( const QwtText &title, QWidget * = NULL );

    virtual ~QwtPlot();

    void applyProperties( const QString & );
    QString grabProperties() const;

    void setAutoReplot( bool on = true );
    bool autoReplot() const;

    // Layout

    void setPlotLayout( QwtPlotLayout * );

    QwtPlotLayout *plotLayout();
    const QwtPlotLayout *plotLayout() const;

    // Title

    void setTitle( const QString & );
    void setTitle( const QwtText &t );
    QwtText title() const;

    QwtTextLabel *titleLabel();
    const QwtTextLabel *titleLabel() const;

    // Footer

    void setFooter( const QString & );
    void setFooter( const QwtText &t );
    QwtText footer() const;

    QwtTextLabel *footerLabel();
    const QwtTextLabel *footerLabel() const;

    // Canvas

    void setCanvas( QWidget * );

    QWidget *canvas();
    const QWidget *canvas() const;

    void setCanvasBackground( const QBrush & );
    QBrush canvasBackground() const;

    virtual QwtScaleMap canvasMap( int axisPos, int id = 0 ) const;

    double invTransform( int axisPos, int pos ) const;
    double invTransform( int axisPos, int id, int pos ) const;

    double transform( int axisPos, double value ) const;
    double transform( int axisPos, int id, double value ) const;

    // Axes

    int axisCount( int axisPos ) const;
    bool isAxisValid( int axisPos, int id ) const;

#if QWT_COMPAT
    QwtScaleEngine *axisScaleEngine( int axisPos, int id = 0 );
    const QwtScaleEngine *axisScaleEngine( int axisPos, int id = 0 ) const;
#else
    QwtScaleEngine *axisScaleEngine( int axisPos, int id );
    const QwtScaleEngine *axisScaleEngine( int axisPos, int id ) const;
#endif

#if QWT_COMPAT
    void setAxisScaleEngine( int axisPos, QwtScaleEngine * );
#endif
    void setAxisScaleEngine( int axisPos, int id, QwtScaleEngine * );

#if QWT_COMPAT
    void setAxisAutoScale( int axisPos, bool on = true );
#endif
    void setAxisAutoScale( int axisPos, int id, bool on = true );
#if QWT_COMPAT
    bool axisAutoScale( int axisPos, int id = 0 ) const;
#else
    bool axisAutoScale( int axisPos, int id ) const;
#endif

#if QWT_COMPAT
    void setAxisVisible( int axisPos, bool on = true );
#endif
    void setAxisVisible( int axisPos, int id, bool on = true );
#if QWT_COMPAT
    bool isAxisVisible( int axisPos, int id = 0 ) const;
#else
    bool isAxisVisible( int axisPos, int id ) const;
#endif

#if QWT_COMPAT
    void setAxisFont( int axisPos, const QFont & );
#endif
    void setAxisFont( int axisPos, int id, const QFont & );
#if QWT_COMPAT
    QFont axisFont( int axisPos, int id = 0 ) const;
#else
    QFont axisFont( int axisPos, int id = 0 ) const;
#endif

#if QWT_COMPAT
    void setAxisScale( int axisPos, double min, double max, double step = 0 );
#endif
    void setAxisScaleDiv( int axisPos, int pos, double min, double max, double step = 0 );
    void setAxisScaleDiv( int axisPos, int pos, const QwtScaleDiv & );
#if QWT_COMPAT
    void setAxisScaleDiv( int axisPos, const QwtScaleDiv & );
#endif
    void setAxisScaleDraw( int axisPos, int pos, QwtScaleDraw * );
#if QWT_COMPAT
    void setAxisScaleDraw( int axisPos, QwtScaleDraw * );
#endif

#if QWT_COMPAT
    double axisStepSize( int axisPos, int id = 0 ) const;
    QwtInterval axisInterval( int axisPos, int id = 0 ) const;
#else
    double axisStepSize( int axisPos, int id ) const;
    QwtInterval axisInterval( int axisPos, int id ) const;
#endif

#if QWT_COMPAT
    const QwtScaleDiv &axisScaleDiv( int axisPos, int id = 0 ) const;

    const QwtScaleDraw *axisScaleDraw( int axisPos, int id = 0 ) const;
    QwtScaleDraw *axisScaleDraw( int axisPos, int id = 0 );

    const QwtScaleWidget *axisWidget( int axisPos, int id = 0 ) const;
    QwtScaleWidget *axisWidget( int axisPos, int id = 0 );
#else
    const QwtScaleDiv &axisScaleDiv( int axisPos, int id ) const;

    const QwtScaleDraw *axisScaleDraw( int axisPos, int id ) const;
    QwtScaleDraw *axisScaleDraw( int axisPos, int id );

    const QwtScaleWidget *axisWidget( int axisPos, int id ) const;
    QwtScaleWidget *axisWidget( int axisPos, int id );
#endif

#if QWT_COMPAT
    void setAxisLabelAlignment( int axisPos, Qt::Alignment );
#endif
    void setAxisLabelAlignment( int axisPos, int pos, Qt::Alignment );
    void setAxisLabelRotation( int axisPos, int pos, double rotation );
#if QWT_COMPAT
    void setAxisLabelRotation( int axisPos, double rotation );
#endif

#if QWT_COMPAT
    void setAxisTitle( int axisPos, const QString & );
    void setAxisTitle( int axisPos, const QwtText & );
#endif
    void setAxisTitle( int axisPos, int pos, const QString & );
    void setAxisTitle( int axisPos, int pos, const QwtText & );
#if QWT_COMPAT
    QwtText axisTitle( int axisPos, int id = 0 ) const;
#else
    QwtText axisTitle( int axisPos, int id ) const;
#endif

#if QWT_COMPAT
    void setAxisMaxMinor( int axisPos, int maxMinor );
#endif
    void setAxisMaxMinor( int axisPos, int id, int maxMinor );
#if QWT_COMPAT
    int axisMaxMinor( int axisPos, int id = 0 ) const;
#else
    int axisMaxMinor( int axisPos, int id ) const;
#endif

#if QWT_COMPAT
    void setAxisMaxMajor( int axisPos, int maxMajor );
#endif
    void setAxisMaxMajor( int axisPos, int id, int maxMajor );
#if QWT_COMPAT
    int axisMaxMajor( int axisPos, int id = 0 ) const;
#else
    int axisMaxMajor( int axisPos, int id ) const;
#endif

    // Legend

    void insertLegend( QwtAbstractLegend *, 
        LegendPosition = QwtPlot::RightLegend, double ratio = -1.0 );

    QwtAbstractLegend *legend();
    const QwtAbstractLegend *legend() const;

    void updateLegend();
    void updateLegend( const QwtPlotItem * );

    // Misc

    virtual QSize sizeHint() const;
    virtual QSize minimumSizeHint() const;

    virtual void updateLayout();
    virtual void drawCanvas( QPainter * );

    void updateAxes();
    void updateCanvasMargins();

    virtual void getCanvasMarginsHint( 
        const QwtScaleMap maps[], const QRectF &canvasRect,
        double &left, double &top, double &right, double &bottom) const;

    virtual bool event( QEvent * );
    virtual bool eventFilter( QObject *, QEvent * );

    virtual void drawItems( QPainter *, const QRectF &,
        const QwtScaleMap maps[NumAxisPositions] ) const;

    virtual QVariant itemToInfo( QwtPlotItem * ) const;
    virtual QwtPlotItem *infoToItem( const QVariant & ) const;

Q_SIGNALS:
    /*!
      A signal indicating, that an item has been attached/detached

      \param plotItem Plot item
      \param on Attached/Detached
     */
    void itemAttached( QwtPlotItem *plotItem, bool on );

    /*!
      A signal with the attributes how to update 
      the legend entries for a plot item.

      \param itemInfo Info about a plot item, build from itemToInfo()
      \param data Attributes of the entries ( usually <= 1 ) for
                  the plot item.

      \sa itemToInfo(), infoToItem(), QwtAbstractLegend::updateLegend()
     */
    void legendDataChanged( const QVariant &itemInfo, 
        const QList<QwtLegendData> &data );

public Q_SLOTS:
    virtual void replot();
    void autoRefresh();

protected:

    virtual void resizeEvent( QResizeEvent *e );

private Q_SLOTS:
    void updateLegendItems( const QVariant &itemInfo,
        const QList<QwtLegendData> &data );

private:
    friend class QwtPlotItem;
    void attachItem( QwtPlotItem *, bool );

    void initScaleData();
    void deleteScaleData();
    void updateScaleDiv();

    void initPlot( const QwtText &title );

    class ScaleData;
    ScaleData *d_scaleData;

    class PrivateData;
    PrivateData *d_data;
};

#if QWT_COMPAT

inline void QwtPlot::setAxisScaleEngine( int axisPos, QwtScaleEngine *engine )
{
    setAxisScaleEngine( axisPos, 0, engine );
}

inline void QwtPlot::setAxisAutoScale( int axisPos, bool on )
{
    setAxisAutoScale( axisPos, 0, on );
}

inline void QwtPlot::setAxisVisible( int axisPos, bool on )
{
    setAxisVisible( axisPos, 0, on );
}

inline void QwtPlot::setAxisFont( int axisPos, const QFont &font )
{
    setAxisFont( axisPos, 0, font );
}

inline void QwtPlot::setAxisScale( int axisPos, double min, double max, double step )
{
    setAxisScaleDiv( axisPos, 0, min, max, step );
}

inline void QwtPlot::setAxisScaleDiv( int axisPos, const QwtScaleDiv &scaleDiv )
{
    setAxisScaleDiv( axisPos, 0, scaleDiv );
}

inline void QwtPlot::setAxisScaleDraw( int axisPos, QwtScaleDraw *scaleDraw )
{
    setAxisScaleDraw( axisPos, 0, scaleDraw );
}

inline void QwtPlot::setAxisLabelAlignment( int axisPos, Qt::Alignment alignment )
{
    setAxisLabelAlignment( axisPos, 0, alignment );
}

inline void QwtPlot::setAxisLabelRotation( int axisPos, double rotation )
{
    setAxisLabelRotation( axisPos, 0, rotation );
}

inline void QwtPlot::setAxisTitle( int axisPos, const QString &title )
{
    setAxisTitle( axisPos, 0, title );
}

inline void QwtPlot::setAxisTitle( int axisPos, const QwtText &title )
{
    setAxisTitle( axisPos, 0, title );
}

inline void QwtPlot::setAxisMaxMinor( int axisPos, int maxMinor )
{
    setAxisMaxMinor( axisPos, 0, maxMinor );
}

inline void QwtPlot::setAxisMaxMajor( int axisPos, int maxMinor )
{
    setAxisMaxMajor( axisPos, 0, maxMinor );
}

inline double QwtPlot::invTransform( int axisPos, int pos ) const
{
    return invTransform( axisPos, 0, pos );
}

inline double QwtPlot::transform( int axisPos, double value ) const
{
    return transform( axisPos, 0, value );
}

#endif

#endif
