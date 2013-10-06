/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 1997   Josef Wilgen
 * Copyright (C) 2002   Uwe Rathmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#include "qwt_plot_layout.h"
#include "qwt_text.h"
#include "qwt_text_label.h"
#include "qwt_scale_widget.h"
#include "qwt_abstract_legend.h"
#include <qscrollbar.h>
#include <qmath.h>

static inline bool qwtIsXAxis( int axisPos ) 
{
    return ( axisPos == QwtPlot::xTop ) || ( axisPos == QwtPlot::xBottom );
}

static inline bool qwtIsYAxis( int axisPos ) 
{
    return ( axisPos == QwtPlot::yLeft ) || ( axisPos == QwtPlot::yRight );
}

class QwtPlotLayoutData
{
public:
    void init( const QwtPlot *, const QRectF &rect );

    bool hasSymmetricYAxes() const;

    struct LegendData
    {
        int frameWidth;
        int hScrollExtent;
        int vScrollExtent;
        QSize hint;
    };

    struct TitleData
    {
        QwtText text;
        int frameWidth;
    };

    struct FooterData
    {
        QwtText text;
        int frameWidth;
    };

    struct ScaleData
    {
        bool isVisible;
        const QwtScaleWidget *scaleWidget;
        QFont scaleFont;
        int start;
        int end;
        int baseLineOffset;
        double tickOffset;
        int dimWithoutTitle;
    };

    struct CanvasData
    {
        int contentsMargins[ QwtPlot::NumAxisPositions ];
    };

    LegendData legendData;
    TitleData titleData;
    FooterData footerData;
    QVector<ScaleData> scaleData[ QwtPlot::NumAxisPositions ];
    CanvasData canvasData;

    int numVisibleScales[ QwtPlot::NumAxisPositions ];
};

/*
  Extract all layout relevant data from the plot components
*/
void QwtPlotLayoutData::init( const QwtPlot *plot, const QRectF &rect )
{
    // legend

    if ( plot->legend() )
    {
        legendData.frameWidth = plot->legend()->frameWidth();
        legendData.hScrollExtent =
            plot->legend()->scrollExtent( Qt::Horizontal );
        legendData.vScrollExtent =
            plot->legend()->scrollExtent( Qt::Vertical );

        const QSize hint = plot->legend()->sizeHint();

        int w = qMin( hint.width(), qFloor( rect.width() ) );
        int h = plot->legend()->heightForWidth( w );
        if ( h <= 0 )
            h = hint.height();

        if ( h > rect.height() )
            w += legendData.hScrollExtent;

        legendData.hint = QSize( w, h );
    }

    // title

    titleData.frameWidth = 0;
    titleData.text = QwtText();

    if ( plot->titleLabel() )
    {
        const QwtTextLabel *label = plot->titleLabel();
        titleData.text = label->text();
        if ( !( titleData.text.testPaintAttribute( QwtText::PaintUsingTextFont ) ) )
            titleData.text.setFont( label->font() );

        titleData.frameWidth = plot->titleLabel()->frameWidth();
    }

    // footer

    footerData.frameWidth = 0;
    footerData.text = QwtText();

    if ( plot->footerLabel() )
    {
        const QwtTextLabel *label = plot->footerLabel();
        footerData.text = label->text();
        if ( !( footerData.text.testPaintAttribute( QwtText::PaintUsingTextFont ) ) )
            footerData.text.setFont( label->font() );

        footerData.frameWidth = plot->footerLabel()->frameWidth();
    }

    // scales

    for ( int axisPos = 0; axisPos < QwtPlot::NumAxisPositions; axisPos++ )
    {
        numVisibleScales[ axisPos ] = 0;

        const int axesCount = plot->axesCount( axisPos );
        scaleData[ axisPos ].resize( axesCount );

        for ( int i = 0; i < axesCount; i++ )
        {
            ScaleData &sclData = scaleData[ axisPos ][i];
            const QwtAxisId axisId( axisPos, i );

            if ( plot->isAxisVisible( axisId ) )
            {
                numVisibleScales[ axisPos ]++;
                const QwtScaleWidget *scaleWidget = plot->axisWidget( axisId );

                sclData.isVisible = true;

                sclData.scaleWidget = scaleWidget;

                sclData.scaleFont = scaleWidget->font();

                sclData.start = scaleWidget->startBorderDist();
                sclData.end = scaleWidget->endBorderDist();

                sclData.baseLineOffset = scaleWidget->margin();
                sclData.tickOffset = scaleWidget->margin();
                if ( scaleWidget->scaleDraw()->hasComponent(
                    QwtAbstractScaleDraw::Ticks ) )
                {
                    sclData.tickOffset +=
                        scaleWidget->scaleDraw()->maxTickLength();
                }

                sclData.dimWithoutTitle = scaleWidget->dimForLength(
                    QWIDGETSIZE_MAX, sclData.scaleFont );

                if ( !scaleWidget->title().isEmpty() )
                {
                    sclData.dimWithoutTitle -=
                        scaleWidget->titleHeightForWidth( QWIDGETSIZE_MAX );
                }
            }
            else
            {
                sclData.isVisible = false;
                sclData.start = 0;
                sclData.end = 0;
                sclData.baseLineOffset = 0;
                sclData.tickOffset = 0.0;
                sclData.dimWithoutTitle = 0;
            }
        }
    }

    // canvas

    plot->canvas()->getContentsMargins( 
        &canvasData.contentsMargins[ QwtPlot::yLeft ], 
        &canvasData.contentsMargins[ QwtPlot::xTop ],
        &canvasData.contentsMargins[ QwtPlot::yRight ],
        &canvasData.contentsMargins[ QwtPlot::xBottom ] );
}

bool QwtPlotLayoutData::hasSymmetricYAxes() const
{
    return numVisibleScales[ QwtPlot::yLeft ] == 
        numVisibleScales[ QwtPlot::yRight ];
}

class QwtPlotLayoutEngine
{
public:
    QwtPlotLayoutEngine():
        d_spacing( 5 )
    {
    }

    QRectF layoutLegend( QwtPlotLayout::Options, 
        const QwtPlotLayoutData::LegendData &, const QRectF & ) const;

    QRectF alignLegend( const QwtPlotLayoutData::LegendData &,
        const QRectF &canvasRect, const QRectF &legendRect ) const;

    void alignScales( QwtPlotLayout::Options, const QwtPlotLayoutData &,
        QRectF &canvasRect, 
        QVector<QRectF> scaleRect[QwtPlot::NumAxisPositions] ) const;

    void expandLineBreaks( QwtPlotLayout::Options options, const QwtPlotLayoutData &,
        const QRectF &rect,
        int &dimTitle, int &dimFooter, int dimAxis[QwtPlot::NumAxisPositions] ) const;

    inline void setSpacing( int spacing ) { d_spacing = spacing; }
    inline int spacing() const { return d_spacing; }

    inline void setAlignCanvas( int axisPos, bool on ) { d_alignCanvas[ axisPos ] = on; }
    inline bool alignCanvas( int axisPos ) const { return d_alignCanvas[ axisPos ]; }

    inline void setCanvasMargin( int axisPos, int margin ) { d_canvasMargin[ axisPos ] = margin; }
    inline int canvasMargin( int axisPos ) const { return d_canvasMargin[ axisPos ]; }

    inline void setLegendPos( QwtPlot::LegendPosition pos ) { d_legendPos = pos; }
    inline QwtPlot::LegendPosition legendPos() const { return d_legendPos; }

    inline void setLegendRatio( double ratio ) { d_legendRatio = ratio; }
    inline double legendRatio() const { return d_legendRatio; }

private:
    QwtPlot::LegendPosition d_legendPos;
    double d_legendRatio;

    int d_canvasMargin[QwtPlot::NumAxisPositions];
    bool d_alignCanvas[QwtPlot::NumAxisPositions];

    int d_spacing;
};

QRectF QwtPlotLayoutEngine::layoutLegend( QwtPlotLayout::Options options,
    const QwtPlotLayoutData::LegendData &legendData, const QRectF &rect ) const
{
    int dim;
    if ( d_legendPos == QwtPlot::LeftLegend
        || d_legendPos == QwtPlot::RightLegend )
    {
        // We don't allow vertical legends to take more than
        // half of the available space.

        dim = qMin( legendData.hint.width(), int( rect.width() * d_legendRatio ) );

        if ( !( options & QwtPlotLayout::IgnoreScrollbars ) )
        {
            if ( legendData.hint.height() > rect.height() )
            {
                // The legend will need additional
                // space for the vertical scrollbar.

                dim += legendData.hScrollExtent;
            }
        }
    }
    else
    {
        dim = qMin( legendData.hint.height(), int( rect.height() * d_legendRatio ) );
        dim = qMax( dim, legendData.vScrollExtent );
    }

    QRectF legendRect = rect;
    switch ( d_legendPos )
    {
        case QwtPlot::LeftLegend:
        {
            legendRect.setWidth( dim );
            break;
        }
        case QwtPlot::RightLegend:
        {
            legendRect.setX( rect.right() - dim );
            legendRect.setWidth( dim );
            break;
        }
        case QwtPlot::TopLegend:
        {
            legendRect.setHeight( dim );
            break;
        }
        case QwtPlot::BottomLegend:
        {
            legendRect.setY( rect.bottom() - dim );
            legendRect.setHeight( dim );
            break;
        }
    }

    return legendRect;
}

QRectF QwtPlotLayoutEngine::alignLegend( 
    const QwtPlotLayoutData::LegendData &legendData,
    const QRectF &canvasRect, const QRectF &legendRect ) const
{
    QRectF alignedRect = legendRect;

    if ( d_legendPos == QwtPlot::BottomLegend
        || d_legendPos == QwtPlot::TopLegend )
    {
        if ( legendData.hint.width() < canvasRect.width() )
        {
            alignedRect.setX( canvasRect.x() );
            alignedRect.setWidth( canvasRect.width() );
        }
    }
    else
    {
        if ( legendData.hint.height() < canvasRect.height() )
        {
            alignedRect.setY( canvasRect.y() );
            alignedRect.setHeight( canvasRect.height() );
        }
    }

    return alignedRect;
}

void QwtPlotLayoutEngine::expandLineBreaks( 
    QwtPlotLayout::Options options, const QwtPlotLayoutData &layoutData, const QRectF &rect,
    int &dimTitle, int &dimFooter, int dimAxis[QwtPlot::NumAxisPositions] ) const
{
    dimTitle = dimFooter = 0;
    for ( int axis = 0; axis < QwtPlot::NumAxisPositions; axis++ )
        dimAxis[axis] = 0;

    int backboneOffset[QwtPlot::NumAxisPositions];
    for ( int axisPos = 0; axisPos < QwtPlot::NumAxisPositions; axisPos++ )
    {
        backboneOffset[ axisPos ] = 0;
        if ( !( options & QwtPlotLayout::IgnoreFrames ) )
            backboneOffset[ axisPos ] += layoutData.canvasData.contentsMargins[ axisPos ];

        if ( !d_alignCanvas[ axisPos ] )
            backboneOffset[ axisPos ] += d_canvasMargin[ axisPos ];
    }

    bool done = false;
    while ( !done )
    {
        done = true;

        // the size for the 4 axis depend on each other. Expanding
        // the height of a horizontal axis will shrink the height
        // for the vertical axis, shrinking the height of a vertical
        // axis will result in a line break what will expand the
        // width and results in shrinking the width of a horizontal
        // axis what might result in a line break of a horizontal
        // axis ... . So we loop as long until no size changes.

        if ( !( ( options & QwtPlotLayout::IgnoreTitle ) || layoutData.titleData.text.isEmpty() ) )
        {
            double w = rect.width();

            if ( !layoutData.hasSymmetricYAxes() )
            {
                // center to the canvas
                w -= dimAxis[QwtPlot::yLeft] + dimAxis[QwtPlot::yRight];
            }

            int d = qCeil( layoutData.titleData.text.heightForWidth( w ) );
            if ( !( options & QwtPlotLayout::IgnoreFrames ) )
                d += 2 * layoutData.titleData.frameWidth;

            if ( d > dimTitle )
            {
                dimTitle = d;
                done = false;
            }
        }

        if ( !( ( options & QwtPlotLayout::IgnoreFooter ) ||
            layoutData.footerData.text.isEmpty() ) )
        {
            double w = rect.width();

            if ( !layoutData.hasSymmetricYAxes() )
            {
                // center to the canvas
                w -= dimAxis[QwtPlot::yLeft] + dimAxis[QwtPlot::yRight];
            }

            int d = qCeil( layoutData.footerData.text.heightForWidth( w ) );
            if ( !( options & QwtPlotLayout::IgnoreFrames ) )
                d += 2 * layoutData.footerData.frameWidth;

            if ( d > dimFooter )
            {
                dimFooter = d;
                done = false;
            }
        }

        for ( int axisPos = 0; axisPos < QwtPlot::NumAxisPositions; axisPos++ )
        {
            const struct QwtPlotLayoutData::ScaleData &scaleData =
                layoutData.scaleData[ axisPos ][ QWT_DUMMY_ID ];

            if ( scaleData.isVisible )
            {
                double length;
                if ( qwtIsXAxis( axisPos ) )
                {
                    length = rect.width() - dimAxis[QwtPlot::yLeft]
                        - dimAxis[QwtPlot::yRight];
                    length -= scaleData.start + scaleData.end;

                    if ( dimAxis[QwtPlot::yRight] > 0 )
                        length -= 1;

                    length += qMin( dimAxis[QwtPlot::yLeft],
                        scaleData.start - backboneOffset[QwtPlot::yLeft] );
                    length += qMin( dimAxis[QwtPlot::yRight],
                        scaleData.end - backboneOffset[QwtPlot::yRight] );
                }
                else 
                {
                    length = rect.height() - dimAxis[QwtPlot::xTop]
                        - dimAxis[QwtPlot::xBottom];
                    length -= scaleData.start + scaleData.end;
                    length -= 1;

                    if ( dimAxis[QwtPlot::xBottom] <= 0 )
                        length -= 1;
                    if ( dimAxis[QwtPlot::xTop] <= 0 )
                        length -= 1;

                    if ( dimAxis[QwtPlot::xBottom] > 0 )
                    {
                        length += qMin(
                            layoutData.scaleData[QwtPlot::xBottom][QWT_DUMMY_ID].tickOffset,
                            double( scaleData.start - backboneOffset[QwtPlot::xBottom] ) );
                    }
                    if ( dimAxis[QwtPlot::xTop] > 0 )
                    {
                        length += qMin(
                            layoutData.scaleData[QwtPlot::xTop][QWT_DUMMY_ID].tickOffset,
                            double( scaleData.end - backboneOffset[QwtPlot::xTop] ) );
                    }

                    if ( dimTitle > 0 )
                        length -= dimTitle + d_spacing;
                }

                int d = scaleData.dimWithoutTitle;
                if ( !scaleData.scaleWidget->title().isEmpty() )
                {
                    d += scaleData.scaleWidget->titleHeightForWidth( qFloor( length ) );
                }


                if ( d > dimAxis[ axisPos ] )
                {
                    dimAxis[ axisPos ] = d;
                    done = false;
                }
            }
        }
    }
}

void QwtPlotLayoutEngine::alignScales( QwtPlotLayout::Options options,
    const QwtPlotLayoutData &layoutData,
    QRectF &canvasRect, QVector<QRectF> scaleRect[QwtPlot::NumAxisPositions] ) const
{
    int backboneOffset[QwtPlot::NumAxisPositions];
    for ( int axisPos = 0; axisPos < QwtPlot::NumAxisPositions; axisPos++ )
    {
        backboneOffset[ axisPos ] = 0;

        if ( !d_alignCanvas[ axisPos ] )
        {
            backboneOffset[ axisPos ] += d_canvasMargin[ axisPos ];
        }

        if ( !( options & QwtPlotLayout::IgnoreFrames ) )
        {
            backboneOffset[ axisPos ] += 
                layoutData.canvasData.contentsMargins[ axisPos ];
        }
    }

    for ( int axisPos = 0; axisPos < QwtPlot::NumAxisPositions; axisPos++ )
    {
        QRectF &axisRect = scaleRect[ axisPos ][ QWT_DUMMY_ID ];

        if ( !axisRect.isValid() )
            continue;

        const int startDist = layoutData.scaleData[ axisPos ][QWT_DUMMY_ID].start;
        const int endDist = layoutData.scaleData[ axisPos ][QWT_DUMMY_ID].end;

        if ( qwtIsXAxis( axisPos ) )
        {
            const QRectF &leftScaleRect = scaleRect[QwtPlot::yLeft][ QWT_DUMMY_ID ];
            const int leftOffset =
                backboneOffset[QwtPlot::yLeft] - startDist;

            if ( leftScaleRect.isValid() )
            {
                const double dx = leftOffset + leftScaleRect.width();
                if ( d_alignCanvas[QwtPlot::yLeft] && dx < 0.0 )
                {
                    /*
                      The axis needs more space than the width
                      of the left scale.
                     */
                    const double cLeft = canvasRect.left(); // qreal -> double
                    canvasRect.setLeft( qMax( cLeft, axisRect.left() - dx ) );
                }
                else
                {
                    const double minLeft = leftScaleRect.left();
                    const double left = axisRect.left() + leftOffset;
                    axisRect.setLeft( qMax( left, minLeft ) );
                }
            }
            else
            {
                if ( d_alignCanvas[QwtPlot::yLeft] && leftOffset < 0 )
                {
                    canvasRect.setLeft( qMax( canvasRect.left(),
                        axisRect.left() - leftOffset ) );
                }
                else
                {
                    if ( leftOffset > 0 )
                        axisRect.setLeft( axisRect.left() + leftOffset );
                }
            }

            const QRectF &rightScaleRect = scaleRect[QwtPlot::yRight][ QWT_DUMMY_ID ];
            const int rightOffset =
                backboneOffset[QwtPlot::yRight] - endDist + 1;

            if ( rightScaleRect.isValid() )
            {
                const double dx = rightOffset + rightScaleRect.width();
                if ( d_alignCanvas[QwtPlot::yRight] && dx < 0 )
                {
                    /*
                      The axis needs more space than the width
                      of the right scale.
                     */
                    const double cRight = canvasRect.right(); // qreal -> double
                    canvasRect.setRight( qMin( cRight, axisRect.right() + dx ) );
                }   

                const double maxRight = rightScaleRect.right();
                const double right = axisRect.right() - rightOffset;
                axisRect.setRight( qMin( right, maxRight ) );
            }
            else
            {
                if ( d_alignCanvas[QwtPlot::yRight] && rightOffset < 0 )
                {
                    canvasRect.setRight( qMin( canvasRect.right(),
                        axisRect.right() + rightOffset ) );
                }
                else
                {
                    if ( rightOffset > 0 )
                        axisRect.setRight( axisRect.right() - rightOffset );
                }
            }
        }
        else // y axes
        {
            const QRectF &bottomScaleRect = scaleRect[QwtPlot::xBottom][ QWT_DUMMY_ID ];
            const int bottomOffset =
                backboneOffset[QwtPlot::xBottom] - endDist + 1;

            if ( bottomScaleRect.isValid() )
            {
                const double dy = bottomOffset + bottomScaleRect.height();
                if ( d_alignCanvas[QwtPlot::xBottom] && dy < 0 )
                {
                    /*
                      The axis needs more space than the height
                      of the bottom scale.
                     */
                    const double cBottom = canvasRect.bottom(); // qreal -> double
                    canvasRect.setBottom( qMin( cBottom, axisRect.bottom() + dy ) );
                }
                else
                {
                    const double maxBottom = bottomScaleRect.top() +
                        layoutData.scaleData[QwtPlot::xBottom][QWT_DUMMY_ID].tickOffset;
                    const double bottom = axisRect.bottom() - bottomOffset;
                    axisRect.setBottom( qMin( bottom, maxBottom ) );
                }
            }
            else
            {
                if ( d_alignCanvas[QwtPlot::xBottom] && bottomOffset < 0 )
                {
                    canvasRect.setBottom( qMin( canvasRect.bottom(),
                        axisRect.bottom() + bottomOffset ) );
                }
                else
                {
                    if ( bottomOffset > 0 )
                        axisRect.setBottom( axisRect.bottom() - bottomOffset );
                }
            }

            const QRectF &topScaleRect = scaleRect[QwtPlot::xTop][ QWT_DUMMY_ID ];
            const int topOffset = backboneOffset[QwtPlot::xTop] - startDist;

            if ( topScaleRect.isValid() )
            {
                const double dy = topOffset + topScaleRect.height();
                if ( d_alignCanvas[QwtPlot::xTop] && dy < 0 )
                {
                    /*
                      The axis needs more space than the height
                      of the top scale.
                     */
                    const double cTop = canvasRect.top(); // qreal -> double
                    canvasRect.setTop( qMax( cTop, axisRect.top() - dy ) );
                }
                else
                {
                    const double minTop = topScaleRect.bottom() -
                        layoutData.scaleData[QwtPlot::xTop][QWT_DUMMY_ID].tickOffset;
                    const double top = axisRect.top() + topOffset;
                    axisRect.setTop( qMax( top, minTop ) );
                }
            }
            else
            {
                if ( d_alignCanvas[QwtPlot::xTop] && topOffset < 0 )
                {
                    canvasRect.setTop( qMax( canvasRect.top(),
                        axisRect.top() - topOffset ) );
                }
                else
                {
                    if ( topOffset > 0 )
                        axisRect.setTop( axisRect.top() + topOffset );
                }
            }
        }
    }

    /*
      The canvas has been aligned to the scale with largest
      border distances. Now we have to realign the other scale.
     */

    for ( int axisPos = 0; axisPos < QwtPlot::NumAxisPositions; axisPos++ )
    {
        for ( int i = 0; i < layoutData.scaleData[ axisPos ].size(); i++ )
        {
            QRectF &sRect = scaleRect[ axisPos ][ i ];
            const QwtPlotLayoutData::ScaleData &scaleData = layoutData.scaleData[ axisPos ][ i ];

            if ( !sRect.isValid() )
                continue;

            if ( qwtIsXAxis( axisPos ) )
            {
                if ( d_alignCanvas[QwtPlot::yLeft] )
                {
                    double y = canvasRect.left() - scaleData.start;
                    if ( !( options & QwtPlotLayout::IgnoreFrames ) )
                        y += layoutData.canvasData.contentsMargins[ QwtPlot::yLeft ];

                    sRect.setLeft( y );
                }
                if ( d_alignCanvas[QwtPlot::yRight] )
                {
                    double y = canvasRect.right() - 1 + scaleData.end;
                    if ( !( options & QwtPlotLayout::IgnoreFrames ) )
                        y -= layoutData.canvasData.contentsMargins[ QwtPlot::yRight ];

                    sRect.setRight( y );
                }

                if ( d_alignCanvas[ axisPos ] )
                {
                    if ( axisPos == QwtPlot::xTop )
                        sRect.setBottom( canvasRect.top() );
                    else
                        sRect.setTop( canvasRect.bottom() );
                }
            }
            else
            {
                if ( d_alignCanvas[QwtPlot::xTop] )
                {
                    double x = canvasRect.top() - scaleData.start;
                    if ( !( options & QwtPlotLayout::IgnoreFrames ) )
                        x += layoutData.canvasData.contentsMargins[ QwtPlot::xTop ];

                    sRect.setTop( x );
                }
                if ( d_alignCanvas[QwtPlot::xBottom] )
                {
                    double x = canvasRect.bottom() - 1 + scaleData.end;
                    if ( !( options & QwtPlotLayout::IgnoreFrames ) )
                        x -= layoutData.canvasData.contentsMargins[ QwtPlot::xBottom ];

                    sRect.setBottom( x );
                }

                if ( d_alignCanvas[ axisPos ] )
                {
                    if ( axisPos == QwtPlot::yLeft )
                        sRect.setRight( canvasRect.left() );
                    else
                        sRect.setLeft( canvasRect.right() );
                }
            }
        }
    }
}

class QwtPlotLayout::PrivateData
{
public:
    QRectF titleRect;
    QRectF footerRect;
    QRectF legendRect;
    QVector<QRectF> scaleRects[QwtPlot::NumAxisPositions];
    QRectF canvasRect;

    QwtPlotLayoutData layoutData;
    QwtPlotLayoutEngine layoutEngine;
};

/*!
  \brief Constructor
 */

QwtPlotLayout::QwtPlotLayout()
{
    d_data = new PrivateData;

    setLegendPosition( QwtPlot::BottomLegend );
    setCanvasMargin( 4 );
    setAlignCanvasToScales( false );

    invalidate();
}

//! Destructor
QwtPlotLayout::~QwtPlotLayout()
{
    delete d_data;
}

/*!
  Change a margin of the canvas. The margin is the space
  above/below the scale ticks. A negative margin will
  be set to -1, excluding the borders of the scales.

  \param margin New margin
  \param axis One of QwtPlot::Axis. Specifies where the position of the margin.
              -1 means margin at all borders.
  \sa canvasMargin()

  \warning The margin will have no effect when alignCanvasToScale() is true
*/

void QwtPlotLayout::setCanvasMargin( int margin, int axisPos )
{
    if ( margin < -1 )
        margin = -1;

    if ( axisPos == -1 )
    {
        for ( axisPos = 0; axisPos < QwtPlot::NumAxisPositions; axisPos++ )
            d_data->layoutEngine.setCanvasMargin( axisPos, margin );
    }
    else if ( axisPos >= 0 && axisPos < QwtPlot::NumAxisPositions )
    {
        d_data->layoutEngine.setCanvasMargin( axisPos, margin );
    }
}

/*!
    \param axisId Axis index
    \return Margin around the scale tick borders
    \sa setCanvasMargin()
*/
int QwtPlotLayout::canvasMargin( int axisId ) const
{
    if ( axisId < 0 || axisId >= QwtPlot::NumAxisPositions )
        return 0;

    return d_data->layoutEngine.canvasMargin( axisId );
}

/*!
  \brief Set the align-canvas-to-axis-scales flag for all axes

  \param on True/False
  \sa setAlignCanvasToScale(), alignCanvasToScale()
*/
void QwtPlotLayout::setAlignCanvasToScales( bool on )
{
    for ( int axisPos = 0; axisPos < QwtPlot::NumAxisPositions; axisPos++ )
        d_data->layoutEngine.setAlignCanvas( axisPos, on );
}

/*!
  Change the align-canvas-to-axis-scales setting. The canvas may:

  - extend beyond the axis scale ends to maximize its size,
  - align with the axis scale ends to control its size.

  The axisId parameter is somehow confusing as it identifies a border
  of the plot and not the axes, that are aligned. F.e when QwtPlot::yLeft
  is set, the left end of the the x-axes ( QwtPlot::xTop, QwtPlot::xBottom )
  is aligned.

  \param axisId Axis index
  \param on New align-canvas-to-axis-scales setting

  \sa setCanvasMargin(), alignCanvasToScale(), setAlignCanvasToScales()
  \warning In case of on == true canvasMargin() will have no effect
*/
void QwtPlotLayout::setAlignCanvasToScale( int axisPos, bool on )
{
    if ( axisPos >= 0 && axisPos < QwtPlot::NumAxisPositions )
        d_data->layoutEngine.setAlignCanvas( axisPos, on );
}

/*!
  Return the align-canvas-to-axis-scales setting. The canvas may:
  - extend beyond the axis scale ends to maximize its size
  - align with the axis scale ends to control its size.

  \param axisId Axis index
  \return align-canvas-to-axis-scales setting
  \sa setAlignCanvasToScale(), setAlignCanvasToScale(), setCanvasMargin()
*/
bool QwtPlotLayout::alignCanvasToScale( int axisPos ) const
{
    if ( axisPos < 0 || axisPos >= QwtPlot::NumAxisPositions )
        return false;

    return d_data->layoutEngine.alignCanvas( axisPos );
}

/*!
  Change the spacing of the plot. The spacing is the distance
  between the plot components.

  \param spacing New spacing
  \sa setCanvasMargin(), spacing()
*/
void QwtPlotLayout::setSpacing( int spacing )
{
    d_data->layoutEngine.setSpacing( qMax( 0, spacing ) );
}

/*!
  \return Spacing
  \sa margin(), setSpacing()
*/
int QwtPlotLayout::spacing() const
{
    return d_data->layoutEngine.spacing();
}

/*!
  \brief Specify the position of the legend
  \param pos The legend's position.
  \param ratio Ratio between legend and the bounding rectangle
               of title, footer, canvas and axes. The legend will be shrunk
               if it would need more space than the given ratio.
               The ratio is limited to ]0.0 .. 1.0]. In case of <= 0.0
               it will be reset to the default ratio.
               The default vertical/horizontal ratio is 0.33/0.5.

  \sa QwtPlot::setLegendPosition()
*/

void QwtPlotLayout::setLegendPosition( QwtPlot::LegendPosition pos, double ratio )
{
    if ( ratio > 1.0 )
        ratio = 1.0;

    switch ( pos )
    {
        case QwtPlot::TopLegend:
        case QwtPlot::BottomLegend:
        {
            if ( ratio <= 0.0 )
                ratio = 0.33;

            d_data->layoutEngine.setLegendRatio( ratio );
            d_data->layoutEngine.setLegendPos( pos );
            break;
        }
        case QwtPlot::LeftLegend:
        case QwtPlot::RightLegend:
        {
            if ( ratio <= 0.0 )
                ratio = 0.5;

            d_data->layoutEngine.setLegendRatio( ratio );
            d_data->layoutEngine.setLegendPos( pos );
            break;
        }
        default:
            break;
    }
}

/*!
  \brief Specify the position of the legend
  \param pos The legend's position. Valid values are
      \c QwtPlot::LeftLegend, \c QwtPlot::RightLegend,
      \c QwtPlot::TopLegend, \c QwtPlot::BottomLegend.

  \sa QwtPlot::setLegendPosition()
*/
void QwtPlotLayout::setLegendPosition( QwtPlot::LegendPosition pos )
{
    setLegendPosition( pos, 0.0 );
}

/*!
  \return Position of the legend
  \sa setLegendPosition(), QwtPlot::setLegendPosition(),
      QwtPlot::legendPosition()
*/
QwtPlot::LegendPosition QwtPlotLayout::legendPosition() const
{
    return d_data->layoutEngine.legendPos();
}

/*!
  Specify the relative size of the legend in the plot
  \param ratio Ratio between legend and the bounding rectangle
               of title, footer, canvas and axes. The legend will be shrunk
               if it would need more space than the given ratio.
               The ratio is limited to ]0.0 .. 1.0]. In case of <= 0.0
               it will be reset to the default ratio.
               The default vertical/horizontal ratio is 0.33/0.5.
*/
void QwtPlotLayout::setLegendRatio( double ratio )
{
    setLegendPosition( legendPosition(), ratio );
}

/*!
  \return The relative size of the legend in the plot.
  \sa setLegendPosition()
*/
double QwtPlotLayout::legendRatio() const
{
    return d_data->layoutEngine.legendRatio();
}

/*!
  \brief Set the geometry for the title

  This method is intended to be used from derived layouts
  overloading activate()

  \sa titleRect(), activate()
 */
void QwtPlotLayout::setTitleRect( const QRectF &rect )
{
    d_data->titleRect = rect;
}

/*!
  \return Geometry for the title
  \sa activate(), invalidate()
*/
QRectF QwtPlotLayout::titleRect() const
{
    return d_data->titleRect;
}

/*!
  \brief Set the geometry for the footer

  This method is intended to be used from derived layouts
  overloading activate()

  \sa footerRect(), activate()
 */
void QwtPlotLayout::setFooterRect( const QRectF &rect )
{
    d_data->footerRect = rect;
}

/*!
  \return Geometry for the footer
  \sa activate(), invalidate()
*/
QRectF QwtPlotLayout::footerRect() const
{
    return d_data->footerRect;
}

/*!
  \brief Set the geometry for the legend

  This method is intended to be used from derived layouts
  overloading activate()

  \param rect Rectangle for the legend

  \sa legendRect(), activate()
 */
void QwtPlotLayout::setLegendRect( const QRectF &rect )
{
    d_data->legendRect = rect;
}

/*!
  \return Geometry for the legend
  \sa activate(), invalidate()
*/
QRectF QwtPlotLayout::legendRect() const
{
    return d_data->legendRect;
}

/*!
  \brief Set the geometry for an axis

  This method is intended to be used from derived layouts
  overloading activate()

  \param axis Axis index
  \param rect Rectangle for the scale

  \sa scaleRect(), activate()
 */
void QwtPlotLayout::setScaleRect( QwtAxisId axisId, const QRectF &rect )
{
    if ( axisId.pos >= 0 && axisId.pos < QwtPlot::NumAxisPositions )
    {
        QVector<QRectF> &scaleRects = d_data->scaleRects[ axisId.pos ];

        if ( axisId.id >= 0 && axisId.id < scaleRects.size() )
            scaleRects[axisId.id] = rect;
    }
}

/*!
  \param axis Axis index
  \return Geometry for the scale
  \sa activate(), invalidate()
*/
QRectF QwtPlotLayout::scaleRect( QwtAxisId axisId ) const
{
    if ( axisId.pos >= 0 && axisId.pos < QwtPlot::NumAxisPositions )
    {
        QVector<QRectF> &scaleRects = d_data->scaleRects[ axisId.pos ];
        if ( axisId.id >= 0 && axisId.id < scaleRects.size() )
            return scaleRects[axisId.id];
    }

    return QRectF();
}

/*!
  \brief Set the geometry for the canvas

  This method is intended to be used from derived layouts
  overloading activate()

  \sa canvasRect(), activate()
 */
void QwtPlotLayout::setCanvasRect( const QRectF &rect )
{
    d_data->canvasRect = rect;
}

/*!
  \return Geometry for the canvas
  \sa activate(), invalidate()
*/
QRectF QwtPlotLayout::canvasRect() const
{
    return d_data->canvasRect;
}

/*!
  Invalidate the geometry of all components.
  \sa activate()
*/
void QwtPlotLayout::invalidate()
{
    d_data->titleRect = d_data->footerRect = 
        d_data->legendRect = d_data->canvasRect = QRectF();

    for ( int axisPos = 0; axisPos < QwtPlot::NumAxisPositions; axisPos++ )
    {
        QVector<QRectF> &scaleRects = d_data->scaleRects[axisPos];

        scaleRects.resize( 1 );
        scaleRects[0] = QRectF();
    }
}

/*!
  \return Minimum size hint
  \param plot Plot widget

  \sa QwtPlot::minimumSizeHint()
*/
QSize QwtPlotLayout::minimumSizeHint( const QwtPlot *plot ) const
{
    class ScaleData
    {
    public:
        ScaleData()
        {
            w = h = minLeft = minRight = tickOffset = 0;
        }

        int w;
        int h;
        int minLeft;
        int minRight;
        int tickOffset;

    } scaleData[QwtPlot::NumAxisPositions];

    int canvasBorder[QwtPlot::NumAxisPositions];

    int fw;
    plot->canvas()->getContentsMargins( &fw, NULL, NULL, NULL );

    int axisPos;
    for ( axisPos = 0; axisPos < QwtPlot::NumAxisPositions; axisPos++ )
    {
        const QwtAxisId axisId( axisPos, QWT_DUMMY_ID );

        if ( plot->isAxisVisible( axisId ) )
        {
            const QwtScaleWidget *scl = plot->axisWidget( axisId );
            ScaleData &sd = scaleData[ axisPos ];

            const QSize hint = scl->minimumSizeHint();
            sd.w = hint.width();
            sd.h = hint.height();
            scl->getBorderDistHint( sd.minLeft, sd.minRight );
            sd.tickOffset = scl->margin();
            if ( scl->scaleDraw()->hasComponent( QwtAbstractScaleDraw::Ticks ) )
                sd.tickOffset += qCeil( scl->scaleDraw()->maxTickLength() );
        }

        canvasBorder[ axisPos ] = fw + d_data->layoutEngine.canvasMargin( axisPos ) + 1;
    }


    for ( axisPos = 0; axisPos < QwtPlot::NumAxisPositions; axisPos++ )
    {
        ScaleData &sd = scaleData[ axisPos ];
        if ( sd.w && qwtIsXAxis( axisPos ) )
        {
            if ( ( sd.minLeft > canvasBorder[QwtPlot::yLeft] )
                && scaleData[QwtPlot::yLeft].w )
            {
                int shiftLeft = sd.minLeft - canvasBorder[QwtPlot::yLeft];
                if ( shiftLeft > scaleData[QwtPlot::yLeft].w )
                    shiftLeft = scaleData[QwtPlot::yLeft].w;

                sd.w -= shiftLeft;
            }
            if ( ( sd.minRight > canvasBorder[QwtPlot::yRight] )
                && scaleData[QwtPlot::yRight].w )
            {
                int shiftRight = sd.minRight - canvasBorder[QwtPlot::yRight];
                if ( shiftRight > scaleData[QwtPlot::yRight].w )
                    shiftRight = scaleData[QwtPlot::yRight].w;

                sd.w -= shiftRight;
            }
        }

        if ( sd.h && qwtIsYAxis( axisPos ) )
        {
            if ( ( sd.minLeft > canvasBorder[QwtPlot::xBottom] ) &&
                scaleData[QwtPlot::xBottom].h )
            {
                int shiftBottom = sd.minLeft - canvasBorder[QwtPlot::xBottom];
                if ( shiftBottom > scaleData[QwtPlot::xBottom].tickOffset )
                    shiftBottom = scaleData[QwtPlot::xBottom].tickOffset;

                sd.h -= shiftBottom;
            }
            if ( ( sd.minLeft > canvasBorder[QwtPlot::xTop] ) &&
                scaleData[QwtPlot::xTop].h )
            {
                int shiftTop = sd.minRight - canvasBorder[QwtPlot::xTop];
                if ( shiftTop > scaleData[QwtPlot::xTop].tickOffset )
                    shiftTop = scaleData[QwtPlot::xTop].tickOffset;

                sd.h -= shiftTop;
            }
        }
    }

    const QWidget *canvas = plot->canvas();

    int left, top, right, bottom;
    canvas->getContentsMargins( &left, &top, &right, &bottom );

    const QSize minCanvasSize = canvas->minimumSize();

    int w = scaleData[QwtPlot::yLeft].w + scaleData[QwtPlot::yRight].w;
    int cw = qMax( scaleData[QwtPlot::xBottom].w, scaleData[QwtPlot::xTop].w )
        + left + 1 + right + 1;
    w += qMax( cw, minCanvasSize.width() );

    int h = scaleData[QwtPlot::xBottom].h + scaleData[QwtPlot::xTop].h;
    int ch = qMax( scaleData[QwtPlot::yLeft].h, scaleData[QwtPlot::yRight].h )
        + top + 1 + bottom + 1;
    h += qMax( ch, minCanvasSize.height() );

    const QwtTextLabel *labels[2];
    labels[0] = plot->titleLabel();
    labels[1] = plot->footerLabel();

    for ( int i = 0; i < 2; i++ )
    {
        const QwtTextLabel *label = labels[i];
        if ( label && !label->text().isEmpty() )
        {
            // we center on the plot canvas.
            const bool centerOnCanvas = plot->axesCount( QwtPlot::yLeft, true ) !=
                plot->axesCount( QwtPlot::yRight, true );

            int labelW = w;
            if ( centerOnCanvas )
            {
                labelW -= scaleData[QwtPlot::yLeft].w
                    + scaleData[QwtPlot::yRight].w;
            }

            int labelH = label->heightForWidth( labelW );
            if ( labelH > labelW ) // Compensate for a long title
            {
                w = labelW = labelH;
                if ( centerOnCanvas )
                {
                    w += scaleData[QwtPlot::yLeft].w
                        + scaleData[QwtPlot::yRight].w;
                }

                labelH = label->heightForWidth( labelW );
            }
            h += labelH + spacing();
        }
    }

    // Compute the legend contribution

    const QwtAbstractLegend *legend = plot->legend();
    if ( legend && !legend->isEmpty() )
    {
        if ( d_data->layoutEngine.legendPos() == QwtPlot::LeftLegend
            || d_data->layoutEngine.legendPos() == QwtPlot::RightLegend )
        {
            int legendW = legend->sizeHint().width();
            int legendH = legend->heightForWidth( legendW );

            if ( legend->frameWidth() > 0 )
                w += spacing();

            if ( legendH > h )
                legendW += legend->scrollExtent( Qt::Horizontal );

            if ( d_data->layoutEngine.legendRatio() < 1.0 )
                legendW = qMin( legendW, int( w / ( 1.0 - d_data->layoutEngine.legendRatio() ) ) );

            w += legendW + spacing();
        }
        else 
        {
            int legendW = qMin( legend->sizeHint().width(), w );
            int legendH = legend->heightForWidth( legendW );

            if ( legend->frameWidth() > 0 )
                h += spacing();

            if ( d_data->layoutEngine.legendRatio() < 1.0 )
                legendH = qMin( legendH, int( h / ( 1.0 - d_data->layoutEngine.legendRatio() ) ) );

            h += legendH + spacing();
        }
    }

    return QSize( w, h );
}

void QwtPlotLayout::update( const QwtPlot *plot,
    const QRectF &plotRect, Options options )
{
    invalidate();

    for ( int axisPos = 0; axisPos < QwtPlot::NumAxisPositions; axisPos++ )
        d_data->scaleRects[ axisPos ].resize( plot->axesCount( axisPos ) );

    activate( plot, plotRect, options );
}

/*!
  \brief Recalculate the geometry of all components.

  \param plot Plot to be layout
  \param plotRect Rectangle where to place the components
  \param options Layout options

  \sa invalidate(), titleRect(), footerRect()
      legendRect(), scaleRect(), canvasRect()
*/
void QwtPlotLayout::activate( const QwtPlot *plot,
    const QRectF &plotRect, Options options )
{
    QRectF rect( plotRect );  // undistributed rest of the plot rect

    // We extract all layout relevant parameters from the widgets,
    // and save them to d_data->layoutData.

    QwtPlotLayoutData &layoutData = d_data->layoutData;
    layoutData.init( plot, rect );

    if ( !( options & IgnoreLegend )
        && plot->legend() && !plot->legend()->isEmpty() )
    {
        d_data->legendRect = d_data->layoutEngine.layoutLegend( 
            options, layoutData.legendData, rect );

        // subtract d_data->legendRect from rect

        const QRegion region( rect.toRect() );
        rect = region.subtracted( d_data->legendRect.toRect() ).boundingRect();

        switch ( d_data->layoutEngine.legendPos() )
        {
            case QwtPlot::LeftLegend:
            {
                rect.setLeft( rect.left() + spacing() );
                break;
            }
            case QwtPlot::RightLegend:
            {
                rect.setRight( rect.right() - spacing() );
                break;
            }
            case QwtPlot::TopLegend:
            {
                rect.setTop( rect.top() + spacing() );
                break;
            }
            case QwtPlot::BottomLegend:
            {
                rect.setBottom( rect.bottom() - spacing() );
                break;
            }
        }
    }

    /*
     +---+-----------+---+
     |       Title       |
     +---+-----------+---+
     |   |   Axis    |   |
     +---+-----------+---+
     | A |           | A |
     | x |  Canvas   | x |
     | i |           | i |
     | s |           | s |
     +---+-----------+---+
     |   |   Axis    |   |
     +---+-----------+---+
     |      Footer       |
     +---+-----------+---+
    */

    // title, footer and axes include text labels. The height of each
    // label depends on its line breaks, that depend on the width
    // for the label. A line break in a horizontal text will reduce
    // the available width for vertical texts and vice versa.
    // expandLineBreaks finds the height/width for title, footer and axes
    // including all line breaks.

    int dimTitle, dimFooter, dimAxes[QwtPlot::NumAxisPositions];
    d_data->layoutEngine.expandLineBreaks( options, 
        d_data->layoutData, rect, dimTitle, dimFooter, dimAxes );

    if ( dimTitle > 0 )
    {
        d_data->titleRect.setRect(
            rect.left(), rect.top(), rect.width(), dimTitle );

        rect.setTop( d_data->titleRect.bottom() + spacing() );

        if ( !layoutData.hasSymmetricYAxes() )
        {
            // if only one of the y axes is missing we align
            // the title centered to the canvas

            d_data->titleRect.setX( rect.left() + dimAxes[QwtPlot::yLeft] );
            d_data->titleRect.setWidth( rect.width()
                - dimAxes[QwtPlot::yLeft] - dimAxes[QwtPlot::yRight] );
        }
    }

    if ( dimFooter > 0 )
    {
        d_data->footerRect.setRect(
            rect.left(), rect.bottom() - dimFooter, rect.width(), dimFooter );

        rect.setBottom( d_data->footerRect.top() - spacing() );

        if ( !layoutData.hasSymmetricYAxes() )
        {
            // if only one of the y axes is missing we align
            // the footer centered to the canvas

            d_data->footerRect.setX( rect.left() + dimAxes[QwtPlot::yLeft] );
            d_data->footerRect.setWidth( rect.width()
                - dimAxes[QwtPlot::yLeft] - dimAxes[QwtPlot::yRight] );
        }
    }

    d_data->canvasRect.setRect(
        rect.x() + dimAxes[QwtPlot::yLeft],
        rect.y() + dimAxes[QwtPlot::xTop],
        rect.width() - dimAxes[QwtPlot::yRight] - dimAxes[QwtPlot::yLeft],
        rect.height() - dimAxes[QwtPlot::xBottom] - dimAxes[QwtPlot::xTop] );

    for ( int axisPos = 0; axisPos < QwtPlot::NumAxisPositions; axisPos++ )
    {
        // set the rects for the axes

        if ( dimAxes[ axisPos ] )
        {
            int dim = dimAxes[ axisPos ];
            QRectF &scaleRect = d_data->scaleRects[ axisPos ][ QWT_DUMMY_ID ];

            scaleRect = d_data->canvasRect;
            switch ( axisPos )
            {
                case QwtPlot::yLeft:
                    scaleRect.setX( d_data->canvasRect.left() - dim );
                    scaleRect.setWidth( dim );
                    break;
                case QwtPlot::yRight:
                    scaleRect.setX( d_data->canvasRect.right() );
                    scaleRect.setWidth( dim );
                    break;
                case QwtPlot::xBottom:
                    scaleRect.setY( d_data->canvasRect.bottom() );
                    scaleRect.setHeight( dim );
                    break;
                case QwtPlot::xTop:
                    scaleRect.setY( d_data->canvasRect.top() - dim );
                    scaleRect.setHeight( dim );
                    break;
            }
            scaleRect = scaleRect.normalized();
        }
    }

    // +---+-----------+---+
    // |  <-   Axis   ->   |
    // +-^-+-----------+-^-+
    // | | |           | | |
    // |   |           |   |
    // | A |           | A |
    // | x |  Canvas   | x |
    // | i |           | i |
    // | s |           | s |
    // |   |           |   |
    // | | |           | | |
    // +-V-+-----------+-V-+
    // |   <-  Axis   ->   |
    // +---+-----------+---+

    // The ticks of the axes - not the labels above - should
    // be aligned to the canvas. So we try to use the empty
    // corners to extend the axes, so that the label texts
    // left/right of the min/max ticks are moved into them.

    d_data->layoutEngine.alignScales( options, d_data->layoutData, 
        d_data->canvasRect, d_data->scaleRects );

    if ( !d_data->legendRect.isEmpty() )
    {
        // We prefer to align the legend to the canvas - not to
        // the complete plot - if possible.

        d_data->legendRect = d_data->layoutEngine.alignLegend( 
            layoutData.legendData, d_data->canvasRect, d_data->legendRect );
    }
}
