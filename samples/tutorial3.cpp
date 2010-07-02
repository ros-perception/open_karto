/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <stack>
#include <map>

#include <kt_Mapper.h>

#ifdef USE_SPA

#include "Spa.h"

#endif

#ifdef USE_PNG

#include <png.h>

#endif

#ifdef USE_XERCES

#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/sax2/Attributes.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>

#include <xercesc/util/XMLDateTime.hpp>
#include <xercesc/util/XMLFloat.hpp>
#include <xercesc/util/XMLDouble.hpp>
#include <xercesc/util/XMLBigInteger.hpp>
#include <xercesc/util/OutOfMemoryException.hpp>

XERCES_CPP_NAMESPACE_USE

class StrX
{
public :
  StrX(const XMLCh* const toTranscode)
  {
    m_LocalForm = XMLString::transcode(toTranscode);
  }

  ~StrX()
  {
    XMLString::release(&m_LocalForm);
  }

  const char* LocalForm() const
  {
    return m_LocalForm;
  }

private :
  char*   m_LocalForm;
};

inline XERCES_STD_QUALIFIER ostream& operator<<(XERCES_STD_QUALIFIER ostream& target, const StrX& toDump)
{
  target << toDump.LocalForm();
  return target;
}

class KartoDatasetReader : public DefaultHandler
{
public:
  KartoDatasetReader()
    : m_ValidVersion(false)
    , m_GotMore(true)
    , m_HaveNewDatasetObject(false)
    , m_Time(0.0)
  {

  }

  virtual ~KartoDatasetReader()
  {
    delete m_pParser;
    m_pParser = NULL;

    delete m_pToken;
    m_pToken = NULL;
  }

public:
  static KartoDatasetReader* OpenDataset(const std::string& rFilename)
  {
    KartoDatasetReader* pDatasetParser = new KartoDatasetReader();
    pDatasetParser->m_pParser = XMLReaderFactory::createXMLReader();

    pDatasetParser->m_pParser->setFeature(XMLUni::fgSAX2CoreValidation, false);
    pDatasetParser->m_pParser->setFeature(XMLUni::fgSAX2CoreNameSpaces, false);
    pDatasetParser->m_pParser->setFeature(XMLUni::fgXercesSchema, false);
    pDatasetParser->m_pParser->setFeature(XMLUni::fgXercesSchemaFullChecking, false);
    pDatasetParser->m_pParser->setFeature(XMLUni::fgSAX2CoreNameSpacePrefixes, false);

    // Set these two lines to true if you want DTD validation!!!!
    pDatasetParser->m_pParser->setFeature(XMLUni::fgXercesSkipDTDValidation, true);
    pDatasetParser->m_pParser->setFeature(XMLUni::fgXercesLoadExternalDTD, false);

    int errorCode = 0;
    try
    {
      pDatasetParser->m_pParser->setContentHandler(pDatasetParser);
      pDatasetParser->m_pParser->setErrorHandler(pDatasetParser);

      pDatasetParser->m_pToken = new XMLPScanToken();
      // Create a progressive scan token
      if (!pDatasetParser->m_pParser->parseFirst(rFilename.c_str(), *pDatasetParser->m_pToken))
      {
        XERCES_STD_QUALIFIER cerr << "scanFirst() failed\n" << XERCES_STD_QUALIFIER endl;
      }
    }
    catch (const OutOfMemoryException&)
    {
      XERCES_STD_QUALIFIER cerr << "OutOfMemoryException" << XERCES_STD_QUALIFIER endl;
      errorCode = 1;
    }
    catch (const XMLException& toCatch)
    {
      XERCES_STD_QUALIFIER cerr << "\nAn error occurred\n  Error: "
        << StrX(toCatch.getMessage())
        << "\n" << XERCES_STD_QUALIFIER endl;
      errorCode = 2;
    }
    catch(std::runtime_error ex)
    {
      errorCode = 3;
    }

    if (errorCode != 0)
    {
      delete pDatasetParser;
      pDatasetParser = NULL;
    }

    return pDatasetParser;
  }

  KartoDatasetReader& operator >> (karto::DatasetObject*& rpDatasetObject)
  {
    rpDatasetObject = NULL;

    while (m_GotMore && !m_pParser->getErrorCount())
    {
      if (HaveNewObject())
      {
        rpDatasetObject = GetCurrentObject();

        return *this;
      }
      
      m_GotMore = m_pParser->parseNext(*m_pToken);
    }

    return *this;
  }

  inline kt_bool HaveNewObject()
  {
    return m_HaveNewDatasetObject;
  }

  inline karto::DatasetObject* GetCurrentObject()
  {
    m_HaveNewDatasetObject = false;
    return m_pCurrentDatasetObject;
  }

  // SAX interface implementation
public:
  virtual void startElement(const XMLCh* const /*uri*/, const XMLCh* const localname, const XMLCh* const qname, const Attributes& attrs)
  {
    std::string elementName = StrX(qname).LocalForm();

    if (elementName == "KartoDataset")
    {
      std::string version;

      // verify version number
      for (XMLSize_t i=0; i<attrs.getLength(); i++)
      {
        std::string attribute = StrX(attrs.getLocalName(i)).LocalForm();
        if (attribute == "version")
        {
          version = StrX(attrs.getValue(i)).LocalForm();
        }
      }

      if (version == "1.2")
      {
        m_ValidVersion = true;
      }
    }

    if (m_ValidVersion == false)
    {
      throw std::runtime_error("Unsupported file format");
    }

    if (elementName == "LaserRangeFinder")
    {
      std::string name = "";
      std::string type = "Custom";
      kt_int32s id = -1;
      kt_int32s parentId = -1;

      for (XMLSize_t i=0; i<attrs.getLength(); i++)
      {
        std::string attribute = StrX(attrs.getLocalName(i)).LocalForm();
        if (attribute == "name")
        {
          name = StrX(attrs.getValue(i)).LocalForm();
        }
        else if (attribute == "id")
        {
          id = XMLBigInteger(attrs.getValue(i)).intValue();
        }
        else if (attribute == "parentId")
        {
          parentId = XMLBigInteger(attrs.getValue(i)).intValue();
        }
        else if (attribute == "type")
        {
          type = StrX(attrs.getValue(i)).LocalForm();
        }
      }

      karto::LaserRangeFinder* pLaserRangeFinder = NULL;

      if (type == "Custom")
      {
        pLaserRangeFinder = karto::LaserRangeFinder::CreateLaserRangeFinder(name, karto::LaserRangeFinder_Custom, id);
      }
      else if (type == "Sick_LMS100")
      {
        pLaserRangeFinder = karto::LaserRangeFinder::CreateLaserRangeFinder(name, karto::LaserRangeFinder_Sick_LMS100, id);
      }
      else if (type == "Sick_LMS200")
      {
        pLaserRangeFinder = karto::LaserRangeFinder::CreateLaserRangeFinder(name, karto::LaserRangeFinder_Sick_LMS200, id);
      }
      else if (type == "Sick_LMS291")
      {
        pLaserRangeFinder = karto::LaserRangeFinder::CreateLaserRangeFinder(name, karto::LaserRangeFinder_Sick_LMS291, id);
      }
      else if (type == "Hokuyo_UTM_30LX")
      {
        pLaserRangeFinder = karto::LaserRangeFinder::CreateLaserRangeFinder(name, karto::LaserRangeFinder_Hokuyo_UTM_30LX, id);
      }
      else if (type == "Hokuyo_URG_04LX")
      {
        pLaserRangeFinder = karto::LaserRangeFinder::CreateLaserRangeFinder(name, karto::LaserRangeFinder_Hokuyo_URG_04LX, id);
      }

      assert(pLaserRangeFinder != NULL);

      m_CurrentLaserRangeFinderId = id;
      m_LaserRangeFinders[id] = pLaserRangeFinder;
    }
    else if (elementName == "LocalizedRangeScan")
    {
      m_CurrentLaserRangeFinderId = -1;

      // verify version number
      for (XMLSize_t i=0; i<attrs.getLength(); i++)
      {
        std::string attribute = StrX(attrs.getLocalName(i)).LocalForm();
        if (attribute == "parentId")
        {
          m_CurrentLaserRangeFinderId = XMLBigInteger(attrs.getValue(i)).intValue();
        }
      }
    }
    else if (elementName == "RangeReadings")
    {
      m_CurrentRangeReadings.clear();
    }
    else if (elementName == "OdometricPose")
    {
      m_CurrentPose = karto::Pose2();
    }
    else if (elementName == "DatasetInfo")
    {
      m_pDatasetInfo = new karto::DatasetInfo();
    }
    else if (elementName == "Parameters")
    {
      std::string name;
      // verify version number
      for (XMLSize_t i=0; i<attrs.getLength(); i++)
      {
        std::string attribute = StrX(attrs.getLocalName(i)).LocalForm();
        if (attribute == "name")
        {
          name = StrX(attrs.getValue(i)).LocalForm();
        }
      }

      m_pParameters = new karto::Parameters(name);
    }

    m_ElementName.push(elementName);
  }

  virtual void endElement(const XMLCh* const /*uri*/, const XMLCh* const localname, const XMLCh* const qname)
  {
    std::string elementName = StrX(qname).LocalForm();

    ParseCharacters(elementName);

    if (elementName == "LocalizedRangeScan")
    {
      karto::LocalizedRangeScan* pLocalizedRangeScan = new karto::LocalizedRangeScan(GetLaserRangeFinder(m_CurrentLaserRangeFinderId), m_CurrentRangeReadings);
      pLocalizedRangeScan->SetOdometricPose(m_CurrentPose);
      pLocalizedRangeScan->SetCorrectedPose(m_CurrentPose);
      pLocalizedRangeScan->SetTime(m_Time);

      m_HaveNewDatasetObject = true;

      m_pCurrentDatasetObject = pLocalizedRangeScan;

      m_Time = 0.0;
    }
    else if (elementName == "LaserRangeFinder")
    {
      karto::LaserRangeFinder* pLaserRangeFinder = m_LaserRangeFinders[m_CurrentLaserRangeFinderId];
    
      pLaserRangeFinder->SetOffsetPose(m_CurrentPose);

      pLaserRangeFinder->Validate();
    }
    else if (elementName == "DatasetInfo")
    {
      m_pCurrentDatasetObject = m_pDatasetInfo;

      m_HaveNewDatasetObject = true;
    }
    else if (elementName == "Parameters")
    {
      m_pCurrentDatasetObject = m_pParameters;

      m_HaveNewDatasetObject = true;
    }

    m_ElementName.pop();
  }

  /**
   * For large string blocks Xerces sometimes decides to split the text into chunks!
   */
  virtual void characters(const XMLCh* const chars, const XMLSize_t charsLength)
  {
    // make copy of chars and trim
    XMLCh* tmpStrValue = XMLString::replicate(chars);
    XMLString::trim(tmpStrValue);

    if (XMLString::stringLen(tmpStrValue) > 0)
    {
      m_CharacterBuffer.push_back( std::pair< XMLCh*, int>(XMLString::replicate(chars), charsLength) );
    }

    XMLString::release(&tmpStrValue);
  }

private:
  void ParseCharacters(const std::string& rElementName)
  {
    if (m_CharacterBuffer.size() > 0)
    {
      // get string from cached character buffer
      XMLCh* pCharacters = NULL;

      // find total buffer size
      int bufferSize = 0;
      for (std::vector< std::pair< XMLCh*, int > >::iterator iter = m_CharacterBuffer.begin(); iter != m_CharacterBuffer.end(); iter++)
      {
        bufferSize+= iter->second;
      }

      pCharacters = (XMLCh*)XMLPlatformUtils::fgMemoryManager->allocate(sizeof(XMLCh) * (bufferSize + 1));
      pCharacters[0] = NULL;

      for (std::vector< std::pair< XMLCh*, int > >::iterator iter = m_CharacterBuffer.begin(); iter != m_CharacterBuffer.end(); iter++)
      {
        XMLString::catString(pCharacters, iter->first);
      }

      // parse string
      std::string elementName = m_ElementName.top();
      std::string parentName = LookupElementInState(1);

      if (elementName == "RangeReadings")
      {
        BaseRefVectorOf<XMLCh>* pElements = XMLString::tokenizeString(pCharacters, XMLPlatformUtils::fgMemoryManager);

        XMLSize_t size = pElements->size();
        for (XMLSize_t i=0; i<size; i++)
        {
          m_CurrentRangeReadings.push_back(atof(StrX(pElements->elementAt(i)).LocalForm()));
        }

        delete pElements;
      }
      else if (elementName == "Time")
      {
        try
        {
          XMLDouble xmlDouble(pCharacters);

          m_Time = xmlDouble.getValue();
        }
        catch (...)
        {
          m_Time = 0.0;
        }
      }

      if (parentName == "OdometricPose" || parentName == "OffsetPose")
      {
        BaseRefVectorOf<XMLCh>* pElements = XMLString::tokenizeString(pCharacters, XMLPlatformUtils::fgMemoryManager);

        if (elementName == "Position")
        {
          m_CurrentPose.SetX(XMLDouble(pElements->elementAt(0)).getValue());
          m_CurrentPose.SetY(XMLDouble(pElements->elementAt(1)).getValue());
        }
        else if (elementName == "Orientation")
        {
          karto::Quaternion orientation;
          orientation.SetX(XMLDouble(pElements->elementAt(0)).getValue());
          orientation.SetY(XMLDouble(pElements->elementAt(1)).getValue());
          orientation.SetZ(XMLDouble(pElements->elementAt(2)).getValue());
          orientation.SetW(XMLDouble(pElements->elementAt(3)).getValue());

          // calculates heading from orientation
          kt_double heading, t1, t2;
          orientation.ToEulerAngles(heading, t1, t2);

          m_CurrentPose.SetHeading(heading);
        }

        delete pElements;
      }
      else if (parentName == "LaserRangeFinder")
      {
        karto::LaserRangeFinder* pLaserRangeFinder = m_LaserRangeFinders[m_CurrentLaserRangeFinderId];

        karto::AbstractParameter* pParameter = pLaserRangeFinder->GetParameterManager()->Get(elementName);
        if(pParameter != NULL)
        {
          pParameter->SetValueFromString(StrX(pCharacters).LocalForm());
        }
        else
        {
          std::cout << "Unable to set LaserRangeFinder parameter: " << elementName << std::endl;
        }
      }
      else if (parentName == "DatasetInfo")
      {
        karto::AbstractParameter* pParameter = m_pDatasetInfo->GetParameterManager()->Get(elementName);
        if(pParameter != NULL)
        {
          pParameter->SetValueFromString(StrX(pCharacters).LocalForm());
        }
        else
        {
          std::cout << "Unable to set DatasetInfo parameter: " << elementName << std::endl;
        }
      }
      else if (parentName == "Parameters")
      {
        m_pParameters->GetParameterManager()->Add(new karto::Parameter<std::string>(elementName, StrX(pCharacters).LocalForm()));
      }

      // clear cached character buffer
      for (std::vector< std::pair< XMLCh*, int > >::iterator iter = m_CharacterBuffer.begin(); iter != m_CharacterBuffer.end(); iter++)
      {
        XMLString::release(&(iter->first));
      }

      m_CharacterBuffer.clear();

      XMLPlatformUtils::fgMemoryManager->deallocate(pCharacters);
    }
  }

  std::string LookupElementInState(kt_int16u level)
  {
    std::string element;

    std::stack<std::string> pushBack;
    for (int i=0; i<level; i++)
    {
      pushBack.push(m_ElementName.top());
      m_ElementName.pop();
    }

    element = m_ElementName.top();

    while (pushBack.size() > 0)
    {
      m_ElementName.push(pushBack.top());
      pushBack.pop();
    }

    return element;
  }

  karto::LaserRangeFinder* GetLaserRangeFinder(kt_int32s id)
  {
    return dynamic_cast<karto::LaserRangeFinder*>(m_LaserRangeFinders[m_CurrentLaserRangeFinderId]);
  }

private:
  XERCES_CPP_NAMESPACE::XMLPScanToken* m_pToken;
  XERCES_CPP_NAMESPACE::SAX2XMLReader* m_pParser;

  kt_bool m_ValidVersion;

  kt_bool m_GotMore;

  std::stack<std::string> m_ElementName;
  std::vector< std::pair< XMLCh*, int > > m_CharacterBuffer;

  std::map<kt_int32s, karto::LaserRangeFinder*> m_LaserRangeFinders;

  kt_int32s m_CurrentLaserRangeFinderId;
  std::vector<kt_double> m_CurrentRangeReadings;

  karto::DatasetObject* m_pCurrentDatasetObject;
  karto::DatasetInfo* m_pDatasetInfo;
  karto::Parameters* m_pParameters;

  karto::Pose2 m_CurrentPose;

  kt_bool m_HaveNewDatasetObject;
  kt_double m_Time;
};

#endif // USE_XERCES

#ifdef USE_PNG

void png_write_ostream(png_structp png_ptr, png_bytep data, png_size_t length)
{
  std::ostream *stream = (std::ostream*)png_get_io_ptr(png_ptr); 
  stream->write((char*)data,length); 
}

void png_flush_ostream(png_structp png_ptr)
{
  std::ostream *stream = (std::ostream*)png_get_io_ptr(png_ptr);
  stream->flush();
}

bool WriteOccupancyGrid(const karto::OccupancyGrid* pGrid, const std::string& rFilename)
{
  if (rFilename.size() == 0)
  {
    std::cerr << "WriteOccupancyGrid Error - rFilename is empty." << std::endl;

    return false;
  }

  if (pGrid == NULL)
  {
    std::cerr << "WriteOccupancyGrid Error - pGrid == NULL" << std::endl;

    return false;
  }

  int compression_level = 0;
  std::ofstream ofs(rFilename.c_str(), std::ios::out | std::ios::binary);

  png_structp png = NULL;
  png_infop   info = NULL;
  png_bytep *rows = NULL;

  //Create write structure
  png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (!png)
  {
    std::cerr << "WriteOccupancyGrid Error - Unable to write occupancy grid to file: " << rFilename << std::endl;

    return false;
  }

  //Create info structure
  info = png_create_info_struct(png);
  if (!info)
  {
    std::cerr << "WriteOccupancyGrid Error - Unable to write occupancy grid to file: " << rFilename << std::endl;

    return false;
  }

  //Set custom write function to ostream
  png_set_write_fn(png, &ofs, png_write_ostream, png_flush_ostream);

  //Set compression level
  png_set_compression_level(png, compression_level);

  // Get grid size
  karto::Size2<kt_int32s> size(pGrid->GetWidthStep(), pGrid->GetHeight());

  //Create row data
  int rowIndex = 0;
  rows = new png_bytep[size.GetHeight()];
  for (int i = size.GetHeight()-1; i >= 0; i--, rowIndex++)
  {
    rows[rowIndex] = (png_bytep)pGrid->GetDataPointer(karto::Point2<kt_int32s>(0, i));
  }

  // Write PNG metadata
  png_text metadata[3];
  std::stringstream converter;
  converter.precision(12);
  std::string text;

  converter.str("");
  converter << 1.0/pGrid->GetCoordinateConverter()->GetScale();
  text = converter.str();
  metadata[0].compression = PNG_TEXT_COMPRESSION_NONE;
  metadata[0].key = const_cast<char*>("resolution");
  metadata[0].text = new char[text.size()];
  memcpy(metadata[0].text, text.c_str(), text.size());
  metadata[0].text_length = text.size();

  converter.str("");
  converter << pGrid->GetCoordinateConverter()->GetOffset().GetX();
  text = converter.str();
  metadata[1].compression = PNG_TEXT_COMPRESSION_NONE;
  metadata[1].key = const_cast<char*>("offsetX");
  metadata[1].text = new char[text.size()];
  memcpy(metadata[1].text, text.c_str(), text.size());
  metadata[1].text_length = text.size();

  converter.str("");
  converter << pGrid->GetCoordinateConverter()->GetOffset().GetY();
  text = converter.str();
  metadata[2].compression = PNG_TEXT_COMPRESSION_NONE;
  metadata[2].key = const_cast<char*>("offsetY");
  metadata[2].text = new char[text.size()];
  memcpy(metadata[2].text, text.c_str(), text.size());
  metadata[2].text_length = text.size();

  png_set_text(png, info, metadata, 3);

  delete[] metadata[0].text;
  delete[] metadata[1].text;
  delete[] metadata[2].text;

  //Write header info
  png_set_IHDR(png, info, size.GetWidth(), size.GetHeight(), 8, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
  png_write_info(png, info);

  //Write data
  png_write_image(png, rows);

  //End write
  png_write_end(png, NULL);

  //Cleanup
  png_destroy_write_struct(&png,&info);
  delete [] rows;

  return true;
}

#else

bool WriteOccupancyGrid(const karto::OccupancyGrid* pGrid, const std::string& rFilename)
{
  return false;
}

#endif // USE_PNG


class MyMapperListener : public karto::MapperDebugListener, public karto::MapperLoopClosureListener
{
public:
  MyMapperListener()
  {
  }

  virtual ~MyMapperListener()
  {
  }

public:
  virtual void Info(const std::string& rInfo) const
  {
    std::cout << rInfo << std::endl;
  }

  virtual void Debug(const std::string& rInfo) const
  {
    std::cout << rInfo << std::endl;
  }

  virtual void LoopClosureCheck(const std::string& rInfo) const
  {
    std::cout << rInfo << std::endl;
  }

  virtual void BeginLoopClosure(const std::string& rInfo) const
  {
    std::cout << rInfo << std::endl;
  }

  virtual void EndLoopClosure(const std::string& rInfo) const
  {
    std::cout << rInfo << std::endl;
  }
};

int main(int argc, char **argv)
{
  /////////////////////////////////////
  // Check arguments
  if (argc != 2)
  {
    std::cout << "Usage: tutorial2 <dataset file>\n       tutorial2 map.kxd" << std::endl;
    return 0;
  }

  std::string filename = argv[1];

  karto::Dataset* pDataset = new karto::Dataset();
  karto::Mapper* pMapper = new karto::Mapper();
  MyMapperListener* pListener = new MyMapperListener();
  SpaSolver* pSpaSolver = new SpaSolver();

  pMapper->AddListener(pListener);
  pMapper->SetScanSolver(pSpaSolver);

#ifdef USE_XERCES

  XMLPlatformUtils::Initialize();

  std::cout << "Opening: " << filename << std::endl;
  KartoDatasetReader* pReader = KartoDatasetReader::OpenDataset(filename);
  if (pReader != NULL)
  {
    karto::DatasetObject* pDatasetObject = NULL;

    std::cout << "Processing scans in dataset." << std::endl;
    
    kt_int32s scanCounter = 0;
    do 
    {
      *pReader >> pDatasetObject;

      if (pDatasetObject != NULL)
      {
        karto::LocalizedRangeScan* pScan = dynamic_cast<karto::LocalizedRangeScan*>(pDatasetObject);

        if(pScan != NULL)
        {
          // Add the localized range scan to the dataset
          if (pMapper->Process(pScan))
          {
            std::cout << scanCounter++ << ": Pose: " << pScan->GetOdometricPose() << " Corrected Pose: " << pScan->GetCorrectedPose() << std::endl;
          }
          else
          {
            std::cout << scanCounter++ << ": Pose: " << pScan->GetOdometricPose() << std::endl;
          }
        }

        pDataset->Add(pDatasetObject);
      }

      //if (scanCounter > 10)
      //  break;
    } while (pDatasetObject != NULL);

    std::cout << "Done processing scans." << std::endl;
    
    delete pReader;
  }


  XMLPlatformUtils::Terminate();

#endif

  // Create a map (occupancy grid) from processed scans
  std::cout << "Creating occupancy grid for scans." << std::endl;
  karto::OccupancyGrid* pOccupancyGrid = karto::OccupancyGrid::CreateFromScans(pMapper->GetAllProcessedScans(), 0.03);

  std::string occupancyGridFilename = filename.substr(0, filename.find_last_of(".")) + ".png";
  std::cout << "Writing occupancy grid to " << occupancyGridFilename << std::endl;
  WriteOccupancyGrid(pOccupancyGrid, occupancyGridFilename);
  delete pOccupancyGrid;

  delete pMapper;
  delete pDataset;
  delete pListener;
  delete pSpaSolver;

  return 0;
}
