#ifndef FIGURE_H
#define FIGURE_H
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <tuw_geometry/pose2d.h>

namespace tuw {
class Figure; /// Prototype
using FigurePtr = std::shared_ptr< Figure > ;
using FigureConstPtr = std::shared_ptr< Figure const>;

/**
 * class to visualize information using OpenCV matrices
 **/
class Figure {
    std::string title_;               /// window name
    std::string label_format_x_;      /// label format string
    std::string label_format_y_;      /// label format string
    cv::Mat view_;                    /// canvas
    cv::Mat background_;              /// background data, grid or image
    cv::Matx33d Mw2m_;                /// transformation world to map
    cv::Matx33d Mm2w_;                /// transformation map to world
    std::string background_filename_; /// if empty no file will be used
    int width_pixel_,  height_pixel_; /// dimensions of the canvas in pixel
    double min_x_, max_x_, min_y_, max_y_, rotation_; /// area and rotation of the visualized space
    double grid_scale_x_, grid_scale_y_; /// dimension of the drawn grid, if -1 no grid will be drawn
    double dx_, dy_; /// dimension of the visualized space
    double ox_, oy_; /// image offset
    double mx_, my_; /// offset of the visualized space
    double sx_, sy_; /// scale

    void drawBackground (); /// draws the background image
    void init();            /// initializes the transformation matrices
public:
    /**
     * constructor
     * @param title title of the displayed windows
     **/
    Figure ( const std::string &title );

    /**
     * used to initialize the figure
     * @param width_pixel pixel size of the canvas
     * @param height_pixel pixel size of the canvas
     * @param min_y minimal y of the visualized space
     * @param max_y maximal y of the visualized space
     * @param min_x minimal x of the visualized space
     * @param max_x maximal x of the visualized space
     * @param rotation rotation of the visualized space
     * @param grid_scale_x dimension of the drawn grid, if -1 no grid will be drawn
     * @param grid_scale_y dimension of the drawn grid, if -1 no grid will be drawn
     * @param background_image file name of an image for the background, it can be empty as well
     **/
    void init ( int width_pixel, int height_pixel, double min_y, double max_y, double min_x, double max_x, double rotation = 0, double grid_scale_x = -1, double grid_scale_y = -1, const std::string &background_image = std::string() );
    
    /**
     * @return title of the window
     **/
    const std::string title() const;
    /**
     * can be used to define the x and y label format
     * @param label_x format of the x label, default "x=%f"
     * @param label_y format of the y label, default "y=%f"
     **/
    void setLabel ( const std::string &label_x = std::string("x=%f"), const std::string &label_y = std::string("y=%f"));

    /**
     * @return the matrix related to the foreground canvas
     **/
    const cv::Mat& view() const;
    /**
     * @return the matrix related to the foreground canvas
     **/
    cv::Mat& view();
    /**
     * @return the matrix related to the background canvas
     **/
    const cv::Mat& background() const;
    /**
     * @return the matrix related to the background canvas
     **/
    cv::Mat& background();
    /**
     *  @returns true if the figure is initialized
     **/
    bool initialized(); 
    /**
     * can be used to clone an image into the foreground independent to the image format (gray, color, ...)
     * @param view source image
     **/
    void setView ( const cv::Mat& view );
    /**
     * draws a line given in the visualization space (meter, ....) into the image
     * @param view image
     * @param p0 start point
     * @param p1 end point 
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
    void line ( cv::Mat &view, const Point2D &p0, const Point2D &p1, const cv::Scalar &color, int thickness=1, int lineType = CV_AA );
    /**
     * draws a line given in the visualization space (meter, ....) into the foreground image
     * @param p0 start point
     * @param p1 end point 
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
    void line ( const Point2D &p0, const Point2D &p1, const cv::Scalar &color, int thickness=1, int lineType = CV_AA );
    /**
     * draws a circle given in the visualization space (meter, ....) into the image
     * @param view image
     * @param p location
     * @param radius radius
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
    void circle ( const Point2D &p, int radius, const cv::Scalar &color, int thickness=1, int lineType = CV_AA );
    /**
     * draws a circle given in the visualization space (meter, ....) into the foreground image
     * @param view image
     * @param p location
     * @param radius radius
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
    void circle ( cv::Mat &view, const Point2D &p, int radius, const cv::Scalar &color, int thickness=1, int lineType = CV_AA );
    /**
     * draws a symbol (dot) given in the visualization space (meter, ....) into the image
     * @param view image
     * @param p location
     * @param radius radius
     * @param color color --> @see opencv
     **/
    void symbol ( cv::Mat &view, const Point2D &p, const cv::Scalar &color );
    /**
     * draws a symbol (dot) given in the visualization space (meter, ....)  into the foreground image
     * @param p location
     * @param radius radius
     * @param color color --> @see opencv
     **/
    void symbol ( const Point2D &p, const cv::Scalar &color );
    /**
     * draws a symbol (pose) given in the visualization space (meter, ....) into the image
     * @param view image
     * @param p location
     * @param radius radius
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
    void symbol ( cv::Mat &view, const Pose2D &p, double radius, const cv::Scalar &color, int thickness=1, int lineType = CV_AA );
    /**
     * draws a symbol (pose) given in the visualization space (meter, ....) into the foreground image
     * @param p location
     * @param radius radius
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
    void symbol ( const Pose2D &p, double radius, const cv::Scalar &color, int thickness=1, int lineType = CV_AA );
    /**
     * draws a text (pose) given in the visualization space (meter, ....) into the image
     * @param view image
     * @param text text
     * @param p location
     * @param fontFace fontFace --> @see opencv
     * @param fontScale fontScale --> @see opencv
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
    void putText ( cv::Mat &view, const std::string& text, const Point2D &p, int fontFace = cv::FONT_HERSHEY_PLAIN, double fontScale = 0.6, cv::Scalar color = cv::Scalar ( 128,0,0 ), int thickness=1, int lineType=CV_AA, bool bottomLeftOrigin=false );
    /**
     * draws a text (pose) given in the visualization space (meter, ....) into the foreground image
     * @param text text
     * @param p location
     * @param fontFace fontFace --> @see opencv
     * @param fontScale fontScale --> @see opencv
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
    void putText ( const std::string& text, const Point2D &p, int fontFace = cv::FONT_HERSHEY_PLAIN, double fontScale = 0.6, cv::Scalar color = cv::Scalar ( 128,0,0 ), int thickness=1, int lineType=CV_AA, bool bottomLeftOrigin=false );
   
    /**
     * overwrites the foreground with the background
     **/
    void clear ();

    /**
     * @return transformation matrix from the visualization space to image space (world -> map)
     **/
    const cv::Matx33d  &Mw2m () const;
    /**
     * @return transformation matrix from the image space to visualization space (map -> world)
     **/
    const cv::Matx33d  &Mm2w () const;

    /**
     * transforms a point from the visualization space to image space (world -> map)
     * @param src point in visualization space (world)
     * @return point in image space (map [pixel])
     **/
    Point2D w2m ( const Point2D &src ) const ;
    /**
     * transforms a point from the visualization space to image space (world -> map)
     * @param src point in visualization space (world)
     * @param des point in image space (map [pixel])
     * @return reference to des
     **/
    Point2D &w2m ( const Point2D &src, Point2D &des ) const;
    /**
     * transforms a point from the image space to visualization space (map -> world)
     * @param src point in image space (map [pixel])
     * @return point in visualization space (world)
     **/
    Point2D m2w ( const Point2D &src ) const ;
    /**
     * transforms a point from the image space to visualization space (map -> world)
     * @param src point in image space (map [pixel])
     * @param des  point in visualization space (world)
     * @return reference to des
     **/
    Point2D &m2w ( const Point2D &src, Point2D &des ) const;

    /**
     * @return canvas (image) width 
     **/
    int width () const ;
    /**
     * @return canvas (image) height 
     **/
    int height () const ;
    /**
     * @return computed x scale
     **/
    double scale_x () const ;
    /**
     * @return computed y scale
     **/
    double scale_y () const ;
    /**
     * @return minimal x of the visualized space
     **/
    double min_x () const ;
    /**
     * @return maximal x of the visualized space
     **/
    double max_x () const ;
    /**
     * @return minimal y of the visualized space
     **/
    double min_y () const ;
    /**
     * @return maximal y of the visualized space
     **/
    double max_y () const ;
    
    /// color to use with the drawing functions
    static const cv::Scalar green;      
    static const cv::Scalar green_bright;
    static const cv::Scalar green_dark;
    static const cv::Scalar red;
    static const cv::Scalar blue;
    static const cv::Scalar blue_bright;
    static const cv::Scalar blue_dark;
    static const cv::Scalar orange;
    static const cv::Scalar yellow;
    static const cv::Scalar cyan;
    static const cv::Scalar magenta;
    static const cv::Scalar gray_bright;
    static const cv::Scalar gray;
    static const cv::Scalar black;
    static const cv::Scalar white;
    
};
}
#endif // FIGRUE_H
