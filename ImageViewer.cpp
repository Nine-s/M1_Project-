/*
 * ImageViewer.cpp
 *
 *  Created on: Dec 15, 2016
 *      Author: linh
 */
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <cmath>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <string>
#include <algorithm>
#include <iosfwd>
#include <memory>
#include <utility>
using namespace std;
//using namespace caffe;  // NOLINT(build/namespaces)
using std::string;


#include <QtGui/QApplication>
#include <QtGui/QScrollBar>
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include <QtGui/QPrintDialog>
#include <QtGui/QAction>
#include <QtGui/QMenuBar>
#include <QtGui/QPainter>
#include <QtGui/QCloseEvent>
#include <QtCore/QSettings>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QMessageBox>
#include <QtCore/QDebug>
#include <QtGui/QIcon>
#include <QtGui/qwidget.h>

#include "../imageModel/Point.h"
#include "../imageModel/Line.h"
#include "../imageModel/Edge.h"
#include "../imageModel/Matrix.h"
#include "../io/Reader.h"
#include "../segmentation/Thresholds.h"
#include "../segmentation/Canny.h"
#include "../segmentation/Suzuki.h"
#include "../imageModel/Image.h"

//Classif---------------->
/*#include <../../caffe/caffe/caffe.hpp>
#ifdef USE_OPENCV
#include <../../caffe/opencv2/core/core.hpp>
#include <../../caffe/opencv2/highgui/highgui.hpp>
#include <../../caffe/opencv2/imgproc/imgproc.hpp>
#endif  // USE_OPENCV
*/
#include "../pht/GHTInPoint.h"
#include "../pht/PHTEntry.h"
#include "../pht/PHoughTransform.h"
#include "../pht/ICP.h"

#include "../histograms/ShapeHistogram.h"
#include "../pointInterest/Treatments.h"
#include "../pointInterest/Segmentation.h"
#include "../pointInterest/ProHoughTransform.h"
#include "../pointInterest/LandmarkDetection.h"
#include "../utils/ImageConvert.h"
#include "../utils/Drawing.h"
#include "../utils/Converter.h"
#include "../correlation/CrossCorrelation.h"

#include "../MAELab.h"

#include "ImageViewer.h"

#define absolute_MAELab_path ("/home/kevin/MAELab")
#define relative_caffe_path_from_MAELab "../caffe"

void ImageViewer::createFileMenu()
{
	openAct = new QAction(QIcon("./resources/ico/open.png"), tr("&Open..."),
		this);
	openAct->setShortcuts(QKeySequence::Open);
	openAct->setStatusTip(tr("Open an existing file"));
	connect(openAct, SIGNAL(triggered()), this, SLOT(open()));

	saveAct = new QAction(QIcon("./resources/ico/save.png"), tr("&Save"), this);
	saveAct->setShortcuts(QKeySequence::Save);
	saveAct->setStatusTip(tr("Save the document to disk"));
	saveAct->setEnabled(false);
	connect(saveAct, SIGNAL(triggered()), this, SLOT(save()));

	saveAsAct = new QAction(tr("Save &As..."), this);
	saveAsAct->setShortcuts(QKeySequence::SaveAs);
	saveAsAct->setStatusTip(tr("Save the document under a new name"));
	saveAsAct->setEnabled(false);
	connect(saveAsAct, SIGNAL(triggered()), this, SLOT(saveAs()));

	closeAct = new QAction(tr("&Close"), this);
	closeAct->setShortcut(tr("Ctrl+W"));
	closeAct->setStatusTip(tr("Close this window"));
	connect(closeAct, SIGNAL(triggered()), this, SLOT(close()));

	exitAct = new QAction(tr("E&xit"), this);
	exitAct->setShortcuts(QKeySequence::Quit);
	exitAct->setStatusTip(tr("Exit the application"));
	connect(exitAct, SIGNAL(triggered()), qApp, SLOT(closeAllWindows()));

}
void ImageViewer::createViewMenu()
{
	zoomInAct = new QAction(QIcon("./resources/ico/1uparrow.png"),
		tr("Zoom &In (25%)"), this);
	zoomInAct->setShortcut(tr("Ctrl++"));
	zoomInAct->setEnabled(false);
	connect(zoomInAct, SIGNAL(triggered()), this, SLOT(zoomIn()));

	zoomOutAct = new QAction(QIcon("./resources/ico/1downarrow.png"),
		tr("Zoom &Out (25%)"), this);
	zoomOutAct->setShortcut(tr("Ctrl+-"));
	zoomOutAct->setEnabled(false);
	connect(zoomOutAct, SIGNAL(triggered()), this, SLOT(zoomOut()));

	normalSizeAct = new QAction(tr("&Normal Size"), this);
	normalSizeAct->setShortcut(tr("Ctrl+N"));
	normalSizeAct->setEnabled(false);
	connect(normalSizeAct, SIGNAL(triggered()), this, SLOT(normalSize()));

	fitToWindowAct = new QAction(tr("&Fit to Window"), this);
	fitToWindowAct->setEnabled(false);
	fitToWindowAct->setCheckable(true);
	fitToWindowAct->setShortcut(tr("Ctrl+J"));
	connect(fitToWindowAct, SIGNAL(triggered()), this, SLOT(fitToWindow()));

	displayMLandmarksAct = new QAction(tr("&Display manual landmarks"), this);
	displayMLandmarksAct->setEnabled(false);
	displayMLandmarksAct->setCheckable(true);
	displayMLandmarksAct->setChecked(false);
	connect(displayMLandmarksAct, SIGNAL(triggered()), this,
		SLOT(displayManualLandmarks()));

	displayALandmarksAct = new QAction(tr("&Display estimated landmarks"), this);
	displayALandmarksAct->setEnabled(false);
	displayALandmarksAct->setCheckable(true);
	connect(displayALandmarksAct, SIGNAL(triggered()), this,
		SLOT(displayAutoLandmarks()));

}
void ImageViewer::viewMenuUpdateActions()
{
	zoomInAct->setEnabled(!fitToWindowAct->isChecked());
	zoomOutAct->setEnabled(!fitToWindowAct->isChecked());
	normalSizeAct->setEnabled(!fitToWindowAct->isChecked());
}

void ImageViewer::scaleImage(double factor)
{
	Q_ASSERT(imageLabel->pixmap());
	scaleFactor *= factor;
	imageLabel->resize(scaleFactor * imageLabel->pixmap()->size());

	adjustScrollBar(scrollArea->horizontalScrollBar(), factor);
	adjustScrollBar(scrollArea->verticalScrollBar(), factor);

	zoomInAct->setEnabled(scaleFactor < 3.0);
	zoomOutAct->setEnabled(scaleFactor > 0.333);
}
void ImageViewer::adjustScrollBar(QScrollBar *scrollBar, double factor)
{
	scrollBar->setValue(
		int(
			factor * scrollBar->value()
				+ ((factor - 1) * scrollBar->pageStep() / 2)));
}

void ImageViewer::createHelpMenu()
{
	aboutAct = new QAction(tr("&About MAELab"), this);
	connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));
	aboutClassifAct = new QAction(tr("&About Classification"), this);
	connect(aboutClassifAct, SIGNAL(triggered()), this, SLOT(aboutClassif()));
}
//------------------>
void ImageViewer::createClassificationMenu()
{
	classificationAct = new QAction(tr("&classification MAELab"), this);
	connect(classificationAct, SIGNAL(triggered()), this, SLOT(classif()));
}

void ImageViewer::createSegmentationMenu()
{
	binaryThresholdAct = new QAction(tr("&Binary threshold"), this);
	binaryThresholdAct->setEnabled(false);
	connect(binaryThresholdAct, SIGNAL(triggered()), this, SLOT(binThreshold()));

	cannyAct = new QAction(tr("&Canny result"), this);
	cannyAct->setEnabled(false);
	connect(cannyAct, SIGNAL(triggered()), this, SLOT(cannyAlgorithm()));

	suzukiAct = new QAction(tr("&Suzuki result"), this);
	suzukiAct->setEnabled(false);
	connect(suzukiAct, SIGNAL(triggered()), this, SLOT(suzukiAlgorithm()));

	lineSegmentationAct = new QAction(tr("&View segmentation result"), this);
	lineSegmentationAct->setEnabled(false);
	connect(lineSegmentationAct, SIGNAL(triggered()), this,
		SLOT(lineSegmentation()));
}
void ImageViewer::createLandmarksMenu()
{
	/*phtAct = new QAction(tr("&Probabilistic Hough Transform"), this);
	 phtAct->setEnabled(false);
	 phtAct->setShortcut(tr("Ctrl+P"));
	 connect(phtAct, SIGNAL(triggered()), this, SLOT(pHoughTransform()));*/

	phtPointsAct = new QAction(tr("&Generalizing Hough Transform"), this);
	phtPointsAct->setEnabled(false);
	phtPointsAct->setShortcut(tr("Ctrl+G"));
	connect(phtPointsAct, SIGNAL(triggered()), this, SLOT(gHoughTransform()));

	autoLandmarksAct = new QAction(tr("&Probabilistic Hough Transform"), this);
	autoLandmarksAct->setEnabled(false);
	autoLandmarksAct->setShortcut(tr("Ctrl+L"));
	connect(autoLandmarksAct, SIGNAL(triggered()), this,
		SLOT(extractLandmarks()));

	measureMBaryAct = new QAction(tr("&Measure manual centroid"), this);
	measureMBaryAct->setEnabled(false);
	connect(measureMBaryAct, SIGNAL(triggered()), this, SLOT(measureMBary()));

	measureEBaryAct = new QAction(tr("&Measure estimated centroid"), this);
	measureEBaryAct->setEnabled(false);
	connect(measureEBaryAct, SIGNAL(triggered()), this, SLOT(measureEBary()));

	dirAutoLandmarksAct = new QAction(tr("Compute automatic landmarks on folder"),
		this);
	dirAutoLandmarksAct->setEnabled(false);
	connect(dirAutoLandmarksAct, SIGNAL(triggered()), this,
		SLOT(dirAutoLandmarks()));

	dirCentroidMeasureAct = new QAction(tr("Measure centroid on folder"), this);
	dirCentroidMeasureAct->setEnabled(false);
	connect(dirCentroidMeasureAct, SIGNAL(triggered()), this,
		SLOT(dirCentroidMeasure()));

	dirGenerateDataAct = new QAction(tr("Random generated the data"), this);
	dirGenerateDataAct->setEnabled(false);
	dirGenerateDataAct->setShortcut(tr("Ctrl+D"));
	connect(dirGenerateDataAct, SIGNAL(triggered()), this,
		SLOT(dirGenerateData()));
}
void ImageViewer::createActions()
{
	createFileMenu();
	createViewMenu();
	createHelpMenu();
	createSegmentationMenu();
	createLandmarksMenu();
	createClassificationMenu();
}
void ImageViewer::createMenus()
{
	fileMenu = new QMenu(tr("&File"), this);
	fileMenu->addAction(openAct);
	fileMenu->addAction(saveAct);
	fileMenu->addAction(saveAsAct);
	fileMenu->addSeparator();
	fileMenu->addAction(closeAct);
	fileMenu->addAction(exitAct);

	viewMenu = new QMenu(tr("&View"), this);
	viewMenu->addAction(zoomInAct);
	viewMenu->addAction(zoomOutAct);
	viewMenu->addSeparator();
	viewMenu->addAction(normalSizeAct);
	viewMenu->addAction(fitToWindowAct);
	viewMenu->addSeparator();
	viewMenu->addAction(displayMLandmarksAct);
	viewMenu->addAction(displayALandmarksAct);

	helpMenu = new QMenu(tr("&Helps"), this);
	helpMenu->addAction(aboutAct);
	helpMenu->addAction(aboutClassifAct);

	classifMenu=new QMenu(tr("&Classification"), this);
	classifMenu->addAction(classificationAct);

	segmentationMenu = new QMenu(tr("&Segmentation"), this);
	segmentationMenu->addAction(binaryThresholdAct);
	segmentationMenu->addAction(cannyAct);
	segmentationMenu->addAction(suzukiAct);
	segmentationMenu->addAction(lineSegmentationAct);

	dominantPointMenu = new QMenu(tr("&Landmarks"), this);
	//dominantPointMenu->addAction(phtAct);
	dominantPointMenu->addAction(phtPointsAct);
	dominantPointMenu->addAction(autoLandmarksAct);
	dominantPointMenu->addSeparator();
	dominantPointMenu->addAction(measureMBaryAct);
	dominantPointMenu->addAction(measureEBaryAct);
	dominantPointMenu->addSeparator();
	QMenu* menuDirectory = dominantPointMenu->addMenu(tr("Working on directory"));
	menuDirectory->addAction(dirAutoLandmarksAct);
	menuDirectory->addAction(dirCentroidMeasureAct);
	menuDirectory->addAction(dirGenerateDataAct);

	// add menus to GUI
	menuBar()->addMenu(fileMenu);
	menuBar()->addMenu(viewMenu);
	menuBar()->addMenu(segmentationMenu);
	menuBar()->addMenu(dominantPointMenu);
	menuBar()->addMenu(helpMenu);
	menuBar()->addMenu(classifMenu);
}
void ImageViewer::createToolBars()
{
	fileToolBar = addToolBar(tr("File"));
	fileToolBar->addAction(openAct);
	fileToolBar->addAction(saveAct);

	viewToolBar = addToolBar(tr("View"));
	viewToolBar->addAction(zoomInAct);
	viewToolBar->addAction(zoomOutAct);
}
void ImageViewer::createStatusBar()
{
	statusBar()->showMessage(tr("Ready"));
}
void ImageViewer::activeFunction()
{
	openAct->setEnabled(true);
	saveAct->setEnabled(true);
	saveAsAct->setEnabled(true);

	zoomInAct->setEnabled(true);
	zoomOutAct->setEnabled(true);
	fitToWindowAct->setEnabled(true);
	normalSizeAct->setEnabled(true);
	displayMLandmarksAct->setEnabled(true);
	if (matImage->getListOfAutoLandmarks().size() > 0)
		displayALandmarksAct->setEnabled(true);

	binaryThresholdAct->setEnabled(true);
	cannyAct->setEnabled(true);
	suzukiAct->setEnabled(true);
	lineSegmentationAct->setEnabled(true);

	//phtAct->setEnabled(true);
	phtPointsAct->setEnabled(true);
	autoLandmarksAct->setEnabled(true);
	if (matImage->getListOfManualLandmarks().size() > 0)
		measureMBaryAct->setEnabled(true);
	if (matImage->getListOfAutoLandmarks().size() > 0)
		measureEBaryAct->setEnabled(true);
	dirAutoLandmarksAct->setEnabled(true);
	dirCentroidMeasureAct->setEnabled(true);
	dirGenerateDataAct->setEnabled(true);

	viewMenuUpdateActions();
	if (!fitToWindowAct->isChecked())
		imageLabel->adjustSize();
}

ImageViewer::ImageViewer()
{
	imageLabel = new QLabel;
	imageLabel->setBackgroundRole(QPalette::Base);
	imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
	imageLabel->setScaledContents(true);

	scrollArea = new QScrollArea;
	scrollArea->setBackgroundRole(QPalette::Dark);
	scrollArea->setWidget(imageLabel);
	setCentralWidget(scrollArea);

	createActions();
	createMenus();
	createToolBars();
	createStatusBar();

	setWindowTitle(tr(".: MAELab :."));
	resize(900, 700);

	setWindowIcon(QIcon("./resources/ico/ip.ico"));
	//parameterPanel = NULL;

}

ImageViewer::~ImageViewer()
{
	delete matImage;
	delete parameterAction;
	delete parameterDialog;

	delete imageLabel;
	delete scrollArea;

	// menu
	delete fileMenu;
	delete viewMenu;
	delete segmentationMenu;
	delete dominantPointMenu;
	delete helpMenu;
	delete classifMenu;

	// toolbar
	delete fileToolBar;
	delete viewToolBar;

	//menu action
	delete openAct;
	delete printAct;
	delete saveAct;
	delete saveAsAct;
	delete closeAct;
	delete exitAct;

	delete zoomInAct;
	delete zoomOutAct;
	delete normalSizeAct;
	delete fitToWindowAct;
	delete displayMLandmarksAct;
	delete displayALandmarksAct;

	delete aboutAct;
	delete aboutClassifAct;

	delete binaryThresholdAct;
	delete cannyAct;
	delete suzukiAct;
	delete lineSegmentationAct;

	delete classificationAct;

	//delete phtAct;
	delete phtPointsAct;
	delete autoLandmarksAct;
	delete measureMBaryAct;
	delete measureEBaryAct;
	delete dirAutoLandmarksAct;
	delete dirCentroidMeasureAct;
	delete dirGenerateDataAct;
}
;
void ImageViewer::loadImage(QString fn)
{

	matImage = new Image(fn.toStdString());
	qImage.load(fn);

	imageLabel->setPixmap(QPixmap::fromImage(qImage));
	scaleFactor = 1.0;

	saveAsAct->setEnabled(true);
	activeFunction();

	this->fileName = fn;
	setWindowTitle(tr("Image Viewer - ") + fileName);
	statusBar()->showMessage(tr("File loaded"), 2000);
}
void ImageViewer::loadImage(Image *_matImage, QImage _qImage, QString tt)
{
	matImage = _matImage;
	qImage = _qImage;
	imageLabel->setPixmap(QPixmap::fromImage(qImage));
	scaleFactor = 1.0;

	saveAct->setEnabled(true);
	activeFunction();

	setWindowTitle(tr("Image Viewer - ") + tt);
	statusBar()->showMessage(tr("Finished"), 2000);
}

void ImageViewer::displayLandmarks(Image *image, vector<Point> lms, RGB color)
{
	Point lm;
	for (size_t i = 0; i < lms.size(); i++)
	{
		lm = lms.at(i);
		cout << "\nManual landmark: " << lm.getX() << "\t" << lm.getY();
		fillCircle(*(image->getRGBMatrix()), lm, 5, color);

	}

}
void ImageViewer::classif()
{
  //get and convert filename
  QFileInfo name(fileName);
  name.completeBaseName();
  QString fname=name.fileName();
  string sname=fname.toStdString();

  //get the relative path of the image
  QDir dir absolute_MAELab_path;
  QString my_img_abs = name.absoluteFilePath();
  QString s = dir.relativeFilePath(my_img_abs);
  string imagePath=s.toStdString();

  //store the comand line
  string caffe_path=relative_caffe_path_from_MAELab;
  std::string temp ="./"+caffe_path+"/build/examples/cpp_classification/classification.bin  ui/NN/de.prototxt  ui/NN/azert.caffemodel  ui/NN/mean.binaryproto  ui/NN/label.txt "+imagePath;
  const char* cmd=temp.c_str();

  //execute the command line and store it's result
  char buffer[5000];
  std::string result="";
  FILE *stream=popen(cmd,"r");
  if(stream){
    while(!feof(stream)){
      if (fgets(buffer, 5000, stream)!=NULL){
	result+=buffer;
      }
    }
    pclose(stream);
  }

  //convert the result
  const char* final=result.c_str();

  //display it in a QMessageBox
  QMessageBox::about(this, tr("classification"),tr(final));

}
// =========================== Slots =======================================
void ImageViewer::about()
{
	QMessageBox::about(this, tr("About MAELab"),
		tr(
			"<p><b>MAELab</b> is a software in Computer Vision. It provides the function to "
				"segmentation and detection dominant points.</p>"));
}
void ImageViewer::aboutClassif()
{
	QMessageBox::about(this, tr("About Classification"),
		tr(
			"To use the Classification option, you need to specify some paths in the file : MAELab/ui/ImageViewer.cpp :\n - #define absolute_MAELab_path (line 80)\n - #define relative_caffe_path_from_MAELab (line 81)"));
			//QPixmap img = QPixmap(/NN/beatles.jpg);
			//mybox.setIconPixmap(img);
			//mybox.show();
}
void ImageViewer::open()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
		QDir::currentPath());
	if (!fileName.isEmpty())
	{
		QImage image(fileName);
		if (image.isNull())
		{
			QMessageBox::information(this, tr("MAELab"),
				tr("Cannot load %1.").arg(fileName));
			return;
		}
		if (!this->fileName.isEmpty())
		{
			ImageViewer* other = new ImageViewer;
			other->loadImage(fileName);
			other->move(x() + 40, y() + 40);
			other->show();
		}
		else
		{
			this->loadImage(fileName);
		}
	}
}
void ImageViewer::save()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save image"), ".",
		tr("Image Files (*.jpg)"));
	if (fileName.isEmpty())
	{
		cout << "\nCan not save the image !!";
		return;
	}
	QApplication::setOverrideCursor(Qt::WaitCursor);
	qImage.save(fileName, "JPEG");

	QApplication::restoreOverrideCursor();

	this->fileName = fileName;
	setWindowTitle(tr("MAELab - ") + fileName);

	saveAct->setEnabled(false);
	saveAsAct->setEnabled(true);

	statusBar()->showMessage(tr("File saved"), 2000);
	return;
}
void ImageViewer::saveAs()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save image"), ".",
		tr("Image Files (*.jpg)"));
	if (fileName.isEmpty())
	{
		cout << "\nCan not save the image !!";
		return;
	}
	QApplication::setOverrideCursor(Qt::WaitCursor);
	qImage.save(fileName, "JPEG");
	QApplication::restoreOverrideCursor();

	saveAct->setEnabled(true);

	statusBar()->showMessage(tr("File saved"), 2000);
	return;
}
void ImageViewer::zoomIn()
{
	scaleImage(1.25);
}
void ImageViewer::zoomOut()
{
	scaleImage(0.8);
}
void ImageViewer::normalSize()
{
	imageLabel->adjustSize();
	scaleFactor = 1.0;
}
void ImageViewer::fitToWindow()
{

	bool fitToWindow = fitToWindowAct->isChecked();
	scrollArea->setWidgetResizable(fitToWindow);
	if (!fitToWindow)
	{
		normalSize();
	}
	viewMenuUpdateActions();
}
void ImageViewer::displayManualLandmarks()
{
	cout << "\n Display the manual landmarks.\n";

	bool currentState = displayMLandmarksAct->isChecked();
	if (currentState)
	{
		if (matImage->getListOfManualLandmarks().size() <= 0)
		{
			QMessageBox msgbox;
			msgbox.setText("Select the manual landmarks file.");
			msgbox.exec();

			QString reflmPath = QFileDialog::getOpenFileName(this);
			matImage->readManualLandmarks(reflmPath.toStdString());

		}
		vector<Point> mLandmarks = matImage->getListOfManualLandmarks();
		RGB color;
		color.R = 255;
		color.G = 0;
		color.B = 0;
		displayLandmarks(matImage, mLandmarks, color);
		displayMLandmarksAct->setChecked(true);
		measureMBaryAct->setEnabled(true);
	}
	else
	{
		Image *img = new Image(fileName.toStdString());
		matImage->setRGBMatrix(img->getRGBMatrix());
		displayMLandmarksAct->setChecked(false);
	}

	if (displayALandmarksAct->isChecked())
	{
		vector<Point> aLM = matImage->getListOfAutoLandmarks();
		if (aLM.size() > 0)
		{
			RGB color;
			color.R = 255;
			color.G = 255;
			color.B = 0;
			displayLandmarks(matImage, aLM, color);
			displayALandmarksAct->setChecked(true);
		}
	}

	this->loadImage(matImage, ptrRGBToQImage(matImage->getRGBMatrix()),
		"Display manual landmarks.");
	this->show();
	cout << "\nFinish.\n";
}
void ImageViewer::displayAutoLandmarks()
{
	vector<Point> autoLM = matImage->getListOfAutoLandmarks();
	QMessageBox message;
	if (autoLM.size() <= 0)
	{
		message.setText(
			"Automatic landmarks do not exists. You need to compute them.");
		message.exec();
	}
	else
	{
		bool currentState = displayALandmarksAct->isChecked();
		if (currentState)
		{
			RGB color;
			color.R = 255;
			color.G = 255;
			color.B = 0;
			displayLandmarks(matImage, autoLM, color);
			displayALandmarksAct->setChecked(true);
		}
		else
		{
			Image *img = new Image(fileName.toStdString());
			matImage->setRGBMatrix(img->getRGBMatrix());
			displayALandmarksAct->setChecked(false);
		}

		if (displayMLandmarksAct->isChecked())
		{
			vector<Point> mLM = matImage->getListOfManualLandmarks();
			if (mLM.size() > 0)
			{
				RGB color;
				color.R = 255;
				color.G = 0;
				color.B = 0;
				displayLandmarks(matImage, mLM, color);
				displayMLandmarksAct->setChecked(true);
			}
		}
		this->loadImage(matImage, ptrRGBToQImage(matImage->getRGBMatrix()),
			"Display estimated landmarks.");
	}

}
void ImageViewer::binThreshold()
{
	cout << "\nBinary thresholding...\n";
	float tValue = matImage->getThresholdValue();
	Segmentation tr; // = new Segmentation();
	tr.setRefImage(*matImage);
	ptr_IntMatrix rsMatrix = tr.threshold(tValue, 255);

	ImageViewer *other = new ImageViewer;
	other->loadImage(matImage, ptrIntToQImage(rsMatrix), "Thresholding result");
	other->show();

}
void ImageViewer::cannyAlgorithm()
{
	cout << "\nCanny Algorithm...\n";
	Segmentation tr;
	tr.setRefImage(*matImage);
	vector<Edge> edges = tr.canny();

	RGB color;
	color.R = 255;
	color.G = color.B = 0;
	Edge edgei;
	Point pi;
	int rows = matImage->getGrayMatrix()->getRows();
	int cols = matImage->getGrayMatrix()->getCols();

	for (size_t i = 0; i < edges.size(); i++)
	{
		edgei = edges.at(i);
		for (size_t k = 0; k < edgei.getPoints().size(); k++)
		{
			pi = edgei.getPoints().at(k);
			if (pi.getX() >= 0 && pi.getX() < cols && pi.getY() >= 0
				&& pi.getY() < rows)
			{
				matImage->getRGBMatrix()->setAtPosition(pi.getY(), pi.getX(), color);
			}
		}
	}
	ImageViewer *other = new ImageViewer;
	other->loadImage(matImage, ptrRGBToQImage(matImage->getRGBMatrix()),
		"Canny result");
	other->move(x() - 40, y() - 40);
	other->show();
}
void ImageViewer::suzukiAlgorithm()
{
	cout << "\nSuzuki Algorithm...\n";
	Segmentation tr;
	tr.setRefImage(*matImage);
	vector<Edge> edgesCanny = tr.canny();
	vector<Edge> edges = verifyProcess(edgesCanny);

	RGB color;
	color.R = 255;
	color.G = color.B = 0;
	Edge edgei;
	Point pi;
	int rows = matImage->getGrayMatrix()->getRows();
	int cols = matImage->getGrayMatrix()->getCols();

	for (size_t i = 0; i < edges.size(); i++)
	{
		edgei = edges.at(i);
		for (size_t k = 0; k < edgei.getPoints().size(); k++)
		{
			pi = edgei.getPoints().at(k);
			if (pi.getX() >= 0 && pi.getX() < cols && pi.getY() >= 0
				&& pi.getY() < rows)
			{
				matImage->getRGBMatrix()->setAtPosition(pi.getY(), pi.getX(), color);
			}
		}
	}
	ImageViewer *other = new ImageViewer;
	other->loadImage(matImage, ptrRGBToQImage(matImage->getRGBMatrix()),
		"Canny result");
	other->move(x() - 40, y() - 40);
	other->show();
}
void ImageViewer::lineSegmentation()
{
	cout << "\nDisplay the list of lines." << endl;
	QMessageBox msgbox;
	Segmentation tr;
	tr.setRefImage(*matImage);
	vector<Line> listOfLines = tr.segment();
	RGB color;
	color.R = 255;
	color.G = color.B = 0;
	Line linei;
	for (size_t i = 0; i < listOfLines.size(); i++)
	{
		linei = listOfLines.at(i);
		drawingLine(*(matImage->getRGBMatrix()), linei, color);
	}

	this->loadImage(matImage, ptrRGBToQImage(matImage->getRGBMatrix()),
		"Landmarks result");
	this->show();

	msgbox.setText("Finish");
	msgbox.exec();
}
// ======================================================= Landmarks points ===============================
void ImageViewer::gHoughTransform()
{
	cout << "\n Generalizing hough transform." << endl;
	QMessageBox msgbox;

	// extract list of points of model image by using canny
	msgbox.setText("Select the model image.");
	msgbox.exec();
	QString fileName2 = QFileDialog::getOpenFileName(this);
	if (fileName2.isEmpty())
		return;
	cout << endl << fileName2.toStdString() << endl;
	Image *modelImage = new Image(fileName2.toStdString());

	msgbox.setText("Select the landmark file of model image.");
	msgbox.exec();
	QString reflmPath = QFileDialog::getOpenFileName(this);
	modelImage->readManualLandmarks(reflmPath.toStdString());
	int rows = matImage->getGrayMatrix()->getRows();
	int cols = matImage->getGrayMatrix()->getCols();
	ProHoughTransform tr;
	tr.setRefImage(*modelImage);

	Point ePoint, mPoint;
	double angleDiff;

	/*ptr_IntMatrix newScene = new Matrix<int>(rows, cols, 0);
	 vector<Point> estLandmarks = tr.generalTransform(*matImage, angleDiff, ePoint,
	 mPoint, newScene);
	 cout<<"\nLandmarks: "<<estLandmarks.size()<<endl;*/
	//==================================================================
	LandmarkDetection lmDetect;
	lmDetect.setRefImage(*modelImage);
	vector<Point> estLandmarks = lmDetect.landmarksAutoDectect2(*matImage, 100,
		300);
	cout << "\nNumber of the landmarks: " << estLandmarks.size() << endl;

	RGB color;
	color.G = 0;
	color.B = 0;
	color.R = 0;
	// drawing...
	//matImage->getRGBMatrix()->rotation(ePoint,angleDiff,1,color);
	//fillCircle(*(matImage->getRGBMatrix()), ePoint, 5, color);
	//drawingLine(*(matImage->getRGBMatrix()), sLine, color);

	//drawingLine(*(matImage->getRGBMatrix()), l2, color);

	Point lm;
	color.B = 255;
	for (size_t i = 0; i < estLandmarks.size(); i++)
	{
		lm = estLandmarks.at(i);
		cout << "\n Landmarks " << i + 1 << ": " << lm.getX() << "\t" << lm.getY()
			<< endl;
		fillCircle(*(matImage->getRGBMatrix()), lm, 5, color);
	}
	Point ebary;
	double mCentroid = measureCentroidPoint(estLandmarks, ebary);
	msgbox.setText(
		"<p>Coordinate of bary point: (" + QString::number(ebary.getX()) + ", "
			+ QString::number(ebary.getY()) + ")</p>"
				"<p>Centroid value: " + QString::number(mCentroid) + "</p");
	msgbox.exec();
	//icpmethod(*modelImage,*matImage);
	this->loadImage(matImage, ptrRGBToQImage(matImage->getRGBMatrix()),
		"Landmarks result");
	this->show();
	//delete newScene;
	msgbox.setText("Finish.");
	msgbox.exec();

}

void ImageViewer::extractLandmarks()
{
	cout << "\n Automatic extraction the landmarks.\n";
	QMessageBox msgbox;
	msgbox.setText("Select the model image.");
	msgbox.exec();

	QString fileName2 = QFileDialog::getOpenFileName(this);
	if (fileName2.isEmpty())
		return;
	cout << endl << fileName2.toStdString() << endl;
	Image *modelImage = new Image(fileName2.toStdString());

	msgbox.setText("Select the landmark file of model image.");
	msgbox.exec();
	QString reflmPath = QFileDialog::getOpenFileName(this);
	modelImage->readManualLandmarks(reflmPath.toStdString());

	LandmarkDetection tr;
	tr.setRefImage(*modelImage);

	Point ePoint;
	double angleDiff;
	vector<Point> lms = tr.landmarksAutoDectect(*matImage, Degree, 500, 400, 500,
		ePoint, angleDiff);
	cout << "\nNumber of the landmarks: " << lms.size() << endl;
	RGB color;
	color.R = 255;
	color.G = 255;
	color.B = 0;
	Point lm;
	matImage->rotate(ePoint, angleDiff, 1);
	for (size_t i = 0; i < lms.size(); i++)
	{
		lm = lms.at(i);
		drawingCircle(*(matImage->getRGBMatrix()), lm, 5, color);
	}
	matImage->setAutoLandmarks(lms);
	displayALandmarksAct->setEnabled(true);
	displayALandmarksAct->setChecked(true);
	measureEBaryAct->setEnabled(true);
	this->loadImage(matImage, ptrRGBToQImage(matImage->getRGBMatrix()),
		"Landmarks result");
	this->show();

	msgbox.setText("Finish");
	msgbox.exec();
}
void ImageViewer::measureMBary()
{
	QMessageBox qmessage;
	vector<Point> mLandmarks = matImage->getListOfManualLandmarks();
	if (mLandmarks.size() > 0)
	{
		Point ebary(0, 0);
		double mCentroid = measureCentroidPoint(mLandmarks, ebary);

		qmessage.setText(
			"<p>Coordinate of bary point: (" + QString::number(ebary.getX()) + ", "
				+ QString::number(ebary.getY()) + ")</p>"
					"<p>Centroid value: " + QString::number(mCentroid) + "</p");
	}
	else
	{
		qmessage.setText("The image has not the manual landmarks.");
	}
	qmessage.exec();
}
void ImageViewer::measureEBary()
{
	QMessageBox qmessage;
	vector<Point> mLandmarks = matImage->getListOfAutoLandmarks();
	if (mLandmarks.size() > 0)
	{
		Point ebary(0, 0);
		double mCentroid = measureCentroidPoint(mLandmarks, ebary);

		qmessage.setText(
			"<p>Coordinate of bary point: (" + QString::number(ebary.getX()) + ", "
				+ QString::number(ebary.getY()) + ")</p>"
					"<p>Centroid value: " + QString::number(mCentroid) + "</p");
	}
	else
	{
		qmessage.setText("The image has not the automatic landmarks.");
	}
	qmessage.exec();
}
void ImageViewer::dirAutoLandmarks()
{
	cout << "\n Automatic estimated landmarks on directory." << endl;
	QMessageBox msgbox;

	msgbox.setText("Select the model's landmarks file");
	msgbox.exec();
	QString lpath = QFileDialog::getOpenFileName(this);
	matImage->readManualLandmarks(lpath.toStdString());

	msgbox.setText("Selecte the scene images folder.");
	msgbox.exec();
	QString folder = QFileDialog::getExistingDirectory(this);

	msgbox.setText("Selecte the saving folder.");
	msgbox.exec();
	QString savefolder = QFileDialog::getExistingDirectory(this);

	vector<string> fileNames = readDirectory(folder.toStdString().c_str());
	Point pk;
	vector<Point> esLandmarks;
	Point ePoint(0, 0);
	double angleDiff = 0;
	LandmarkDetection tr;
	tr.setRefImage(*matImage);

	string fileName;
	for (size_t i = 0; i < fileNames.size(); i++)
	{
		fileName = folder.toStdString() + "/" + fileNames.at(i);
		cout << "\n" << fileName << endl;
		Image sceneimage(fileName);

		esLandmarks = tr.landmarksAutoDectect(sceneimage, Degree, 500, 400, 500,
			ePoint, angleDiff);
		if (savefolder != NULL || savefolder != "")
		{
			string saveFile = savefolder.toStdString() + "/" + fileNames.at(i)
				+ ".TPS";
			ofstream inFile(saveFile.c_str());
			inFile << "LM=" << esLandmarks.size() << "\n";
			for (size_t k = 0; k < esLandmarks.size(); k++)
			{
				pk = esLandmarks.at(k);
				inFile << pk.getX() << "\t" << pk.getY() << "\n";
			}
			inFile << "IMAGE=" << saveFile << "\n";
			inFile.close();
		}
	}

	msgbox.setText("Finish");
	msgbox.exec();

}
void ImageViewer::dirCentroidMeasure()
{
	cout << "\n Compute centroid on directory." << endl;

	QMessageBox qmessage;
	qmessage.setText("Select the landmarks folder.");
	qmessage.exec();

	QString lmfolder = QFileDialog::getExistingDirectory(this);

	QString fileName = QFileDialog::getSaveFileName(this, tr("Save file"), ".",
		tr("Measure file (*.txt)"));

	vector<string> fileNames = readDirectory(lmfolder.toStdString().c_str());
	string saveFile;
	ofstream inFile;
	if (fileName != NULL || fileName != "")
	{
		string saveFile = fileName.toStdString();
		inFile.open(saveFile.c_str(), std::ofstream::out);
	}
	for (size_t i = 0; i < fileNames.size(); i++)
	{
		string filePath = lmfolder.toStdString() + "/" + fileNames.at(i);
		//vector<Point> mLandmarks = readTPSFile(filePath.c_str());

		//matImage->setMLandmarks(filePath);
		vector<Point> mLandmarks = matImage->readManualLandmarks(filePath);
		if (mLandmarks.size() > 0)
		{
			Point ebary(0, 0);
			double mCentroid = 0;
			mCentroid = measureCentroidPoint(mLandmarks, ebary);

			if (fileName != NULL || fileName != "")
			{
				inFile << fileNames.at(i) << "\t" << ebary.getX() << "\t"
					<< ebary.getY() << "\t" << mCentroid << "\n";

			}
		}
		else
		{
			if (fileName != NULL || fileName != "")
			{
				inFile << fileNames.at(i) << "\t" << 0 << "\t" << 0 << "\t" << 0
					<< "\n";

			}
		}
		mLandmarks.clear();
	}
	inFile.close();

	qmessage.setText("Finish.");
	qmessage.exec();
}
// generation landmarks with PHT on lines
/*void ImageViewer::dirGenerateData()
 {
 cout << "\n Automatic generate data on directory." << endl;
 QMessageBox msgbox;

 string imageFolder = "/home/linh/Desktop/Temps/md/images";
 string lmFolder = "/home/linh/Desktop/Temps/md/landmarks";
 string saveFolder = "/home/linh/Desktop/results/2017/md/random";
 vector<string> images = readDirectory(imageFolder.c_str());
 vector<string> lms = readDirectory(lmFolder.c_str());
 int nrandom = 0;
 string model;
 string sceneName;
 string lmFile;

 Point pk;
 vector<Point> esLandmarks;
 Point ePoint(0, 0);
 double angleDiff = 0;
 LandmarkDetection tr;
 for (int i = 0; i < 21; i++)
 { // run on 20 images
 nrandom = random(0, (int) images.size());

 string modelName = images.at(nrandom);
 cout << "\n Random and model: " << nrandom << "\t" << modelName << endl;
 model = imageFolder + "/" + images.at(nrandom);
 lmFile = lmFolder + "/" + lms.at(nrandom);
 matImage = new Image(model);
 matImage->readManualLandmarks(lmFile);
 tr.setRefImage(*matImage);
 tr.landmarksOnDir(modelName, imageFolder, images, Degree, 500, 400, 500,
 ePoint, angleDiff, saveFolder);
 }

 msgbox.setText("Finish");
 msgbox.exec();
 }*/
// GHT with points
void ImageViewer::dirGenerateData()
{
	cout << "\n Automatic generate data on directory." << endl;
	QMessageBox msgbox;

	string imageFolder = "/home/linh/Desktop/Temps/md/images";
	string lmFolder = "/home/linh/Desktop/Temps/md/landmarks";
	string saveFolder = "/home/linh/Desktop/est";
	vector<string> images = readDirectory(imageFolder.c_str());
	vector<string> lms = readDirectory(lmFolder.c_str());
	int nrandom = 0;
	string model;
	string sceneName;
	string lmFile;

	Point pk;
	vector<Point> esLandmarks;
	Point ePoint(0, 0);
	LandmarkDetection tr;
	//for (int i = 0; i < 21; i++)
	//{ // run on 20 images
	//nrandom = random(0, (int) images.size());
	nrandom = 26; // use Md 028 as ref
	string modelName = images.at(nrandom);
	cout << "\n Random and model: " << nrandom << "\t" << modelName << endl;
	model = imageFolder + "/" + images.at(nrandom);
	lmFile = lmFolder + "/" + lms.at(nrandom);
	matImage = new Image(model);
	matImage->readManualLandmarks(lmFile);
	tr.setRefImage(*matImage);
	tr.landmarksOnDir2(modelName, imageFolder, images, saveFolder);

	//}

	msgbox.setText("Finish");
	msgbox.exec();
}
