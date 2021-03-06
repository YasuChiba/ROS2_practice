// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <uxr/agent/datareader/DataReader.hpp>
#include <uxr/agent/subscriber/Subscriber.hpp>
#include <uxr/agent/participant/Participant.hpp>
#include <uxr/agent/topic/Topic.hpp>
#include <uxr/agent/client/ProxyClient.hpp>
#include <uxr/agent/utils/TokenBucket.hpp>
#include <uxr/agent/logger/Logger.hpp>

namespace eprosima {
namespace uxr {

std::unique_ptr<DataReader> DataReader::create(
        const dds::xrce::ObjectId& object_id,
        const std::shared_ptr<Subscriber>& subscriber,
        const dds::xrce::DATAREADER_Representation& representation,
        const ObjectContainer& root_objects)
{
    bool created_entity = false;
    uint16_t raw_object_id = conversion::objectid_to_raw(object_id);
    std::shared_ptr<Topic> topic;

    Middleware& middleware = subscriber->get_participant()->get_proxy_client()->get_middleware();
    switch (representation.representation()._d())
    {
        case dds::xrce::REPRESENTATION_BY_REFERENCE:
        {
            const std::string& ref = representation.representation().object_reference();
            uint16_t raw_topic_id;
            if (middleware.create_datareader_by_ref(raw_object_id, subscriber->get_raw_id(), ref, raw_topic_id))
            {
                dds::xrce::ObjectId topic_id = conversion::raw_to_objectid(raw_topic_id, dds::xrce::OBJK_TOPIC);;
                topic = std::dynamic_pointer_cast<Topic>(root_objects.at(topic_id));
                topic->tie_object(object_id);
                created_entity = true;
            }
            break;
        }
        case dds::xrce::REPRESENTATION_AS_XML_STRING:
        {
            const std::string& xml = representation.representation().xml_string_representation();
            uint16_t raw_topic_id;
            if (middleware.create_datareader_by_xml(raw_object_id, subscriber->get_raw_id(), xml, raw_topic_id))
            {
                dds::xrce::ObjectId topic_id = conversion::raw_to_objectid(raw_topic_id, dds::xrce::OBJK_TOPIC);
                topic = std::dynamic_pointer_cast<Topic>(root_objects.at(topic_id));
                topic->tie_object(object_id);
                created_entity = true;
            }
            break;
        }
        default:
            break;
    }

    return (created_entity ? std::unique_ptr<DataReader>(new DataReader(object_id, subscriber, topic)) : nullptr);
}

DataReader::DataReader(
        const dds::xrce::ObjectId& object_id,
        const std::shared_ptr<Subscriber>& subscriber,
        const std::shared_ptr<Topic>& topic)
    : XRCEObject(object_id)
    , subscriber_(subscriber)
    , topic_(topic)
    , reader_{}
{
    subscriber_->tie_object(object_id);
    topic_->tie_object(object_id);
}

DataReader::~DataReader() noexcept
{
    reader_.stop_reading();
    subscriber_->untie_object(get_id());
    topic_->untie_object(get_id());
    subscriber_->get_participant()->get_proxy_client()->get_middleware().delete_datareader(get_raw_id());
}

bool DataReader::matched(
        const dds::xrce::ObjectVariant& new_object_rep) const
{
    /* Check ObjectKind. */
    if ((get_id().at(1) & 0x0F) != new_object_rep._d())
    {
        return false;
    }

    bool rv = false;
    switch (new_object_rep.data_reader().representation()._d())
    {
        case dds::xrce::REPRESENTATION_BY_REFERENCE:
        {
            const std::string& ref = new_object_rep.data_reader().representation().object_reference();
            rv = subscriber_->get_participant()->get_proxy_client()->get_middleware().matched_datareader_from_ref(get_raw_id(), ref);
            break;
        }
        case dds::xrce::REPRESENTATION_AS_XML_STRING:
        {
            const std::string& xml = new_object_rep.data_reader().representation().xml_string_representation();
            rv = subscriber_->get_participant()->get_proxy_client()->get_middleware().matched_datareader_from_xml(get_raw_id(), xml);
            break;
        }
        default:
            break;
    }
    return rv;
}

bool DataReader::read(
        const dds::xrce::READ_DATA_Payload& read_data,
        Reader<bool>::WriteFn write_fn,
        WriteFnArgs& write_args)
{
    dds::xrce::DataDeliveryControl delivery_control;
    if (read_data.read_specification().has_delivery_control())
    {
        delivery_control = read_data.read_specification().delivery_control();
    }
    else
    {
        delivery_control.max_elapsed_time(0);
        delivery_control.max_bytes_per_second(0);
        delivery_control.max_samples(1);
    }

    /* TODO (julianbermudez): implement different data formats.
    switch (read_data.read_specification().data_format())
    {
        case dds::xrce::FORMAT_DATA:
            break;
        case dds::xrce::FORMAT_SAMPLE:
            break;
        case dds::xrce::FORMAT_DATA_SEQ:
            break;
        case dds::xrce::FORMAT_SAMPLE_SEQ:
            break;
        case dds::xrce::FORMAT_PACKED_SAMPLES:
            break;
        default:
            break;
    }
    */

    write_args.client = subscriber_->get_participant()->get_proxy_client();

    using namespace std::placeholders;
    return (reader_.stop_reading() &&
            reader_.start_reading(delivery_control, std::bind(&DataReader::read_fn, this, _1, _2, _3), false, write_fn, write_args));
}

bool DataReader::read_fn(
        bool,
        std::vector<uint8_t>& data,
        std::chrono::milliseconds timeout)
{
    bool rv = false;
    if (subscriber_->get_participant()->get_proxy_client()->get_middleware().read_data(get_raw_id(), data, timeout))
    {
        UXR_AGENT_LOG_MESSAGE(
            UXR_DECORATE_YELLOW("[==>> DDS <<==]"),
            get_raw_id(),
            data.data(),
            data.size());
        rv = true;
    }
    return rv;
}

} // namespace uxr
} // namespace eprosima
