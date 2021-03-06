// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: serial_com_config.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "serial_com_config.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace roborts_serial {

namespace {

const ::google::protobuf::Descriptor* SerialPortConfig_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  SerialPortConfig_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_serial_5fcom_5fconfig_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AssignDesc_serial_5fcom_5fconfig_2eproto() {
  protobuf_AddDesc_serial_5fcom_5fconfig_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "serial_com_config.proto");
  GOOGLE_CHECK(file != NULL);
  SerialPortConfig_descriptor_ = file->message_type(0);
  static const int SerialPortConfig_offsets_[10] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SerialPortConfig, serial_port_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SerialPortConfig, serial_boudrate_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SerialPortConfig, flow_control_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SerialPortConfig, databits_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SerialPortConfig, stopbits_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SerialPortConfig, parity_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SerialPortConfig, is_debug_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SerialPortConfig, is_simulator_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SerialPortConfig, link_column_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SerialPortConfig, link_beam_),
  };
  SerialPortConfig_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      SerialPortConfig_descriptor_,
      SerialPortConfig::default_instance_,
      SerialPortConfig_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SerialPortConfig, _has_bits_[0]),
      -1,
      -1,
      sizeof(SerialPortConfig),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SerialPortConfig, _internal_metadata_),
      -1);
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_serial_5fcom_5fconfig_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      SerialPortConfig_descriptor_, &SerialPortConfig::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_serial_5fcom_5fconfig_2eproto() {
  delete SerialPortConfig::default_instance_;
  delete SerialPortConfig_reflection_;
  delete SerialPortConfig::_default_parity_;
}

void protobuf_AddDesc_serial_5fcom_5fconfig_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AddDesc_serial_5fcom_5fconfig_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\027serial_com_config.proto\022\016roborts_seria"
    "l\"\373\001\n\020SerialPortConfig\022\023\n\013serial_port\030\001 "
    "\002(\t\022\027\n\017serial_boudrate\030\002 \002(\005\022\027\n\014flow_con"
    "trol\030\003 \001(\005:\0010\022\023\n\010databits\030\004 \001(\005:\0018\022\023\n\010st"
    "opbits\030\005 \001(\005:\0011\022\021\n\006parity\030\006 \001(\t:\001N\022\026\n\010is"
    "_debug\030\007 \001(\010:\004true\022\033\n\014is_simulator\030\010 \001(\010"
    ":\005false\022\027\n\013link_column\030\t \001(\001:\00250\022\025\n\tlink"
    "_beam\030\n \001(\001:\00290", 295);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "serial_com_config.proto", &protobuf_RegisterTypes);
  SerialPortConfig::_default_parity_ =
      new ::std::string("N", 1);
  SerialPortConfig::default_instance_ = new SerialPortConfig();
  SerialPortConfig::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_serial_5fcom_5fconfig_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_serial_5fcom_5fconfig_2eproto {
  StaticDescriptorInitializer_serial_5fcom_5fconfig_2eproto() {
    protobuf_AddDesc_serial_5fcom_5fconfig_2eproto();
  }
} static_descriptor_initializer_serial_5fcom_5fconfig_2eproto_;

// ===================================================================

::std::string* SerialPortConfig::_default_parity_ = NULL;
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int SerialPortConfig::kSerialPortFieldNumber;
const int SerialPortConfig::kSerialBoudrateFieldNumber;
const int SerialPortConfig::kFlowControlFieldNumber;
const int SerialPortConfig::kDatabitsFieldNumber;
const int SerialPortConfig::kStopbitsFieldNumber;
const int SerialPortConfig::kParityFieldNumber;
const int SerialPortConfig::kIsDebugFieldNumber;
const int SerialPortConfig::kIsSimulatorFieldNumber;
const int SerialPortConfig::kLinkColumnFieldNumber;
const int SerialPortConfig::kLinkBeamFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

SerialPortConfig::SerialPortConfig()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:roborts_serial.SerialPortConfig)
}

void SerialPortConfig::InitAsDefaultInstance() {
}

SerialPortConfig::SerialPortConfig(const SerialPortConfig& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:roborts_serial.SerialPortConfig)
}

void SerialPortConfig::SharedCtor() {
  ::google::protobuf::internal::GetEmptyString();
  _cached_size_ = 0;
  serial_port_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  serial_boudrate_ = 0;
  flow_control_ = 0;
  databits_ = 8;
  stopbits_ = 1;
  parity_.UnsafeSetDefault(_default_parity_);
  is_debug_ = true;
  is_simulator_ = false;
  link_column_ = 50;
  link_beam_ = 90;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

SerialPortConfig::~SerialPortConfig() {
  // @@protoc_insertion_point(destructor:roborts_serial.SerialPortConfig)
  SharedDtor();
}

void SerialPortConfig::SharedDtor() {
  serial_port_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  parity_.DestroyNoArena(_default_parity_);
  if (this != default_instance_) {
  }
}

void SerialPortConfig::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* SerialPortConfig::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return SerialPortConfig_descriptor_;
}

const SerialPortConfig& SerialPortConfig::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_serial_5fcom_5fconfig_2eproto();
  return *default_instance_;
}

SerialPortConfig* SerialPortConfig::default_instance_ = NULL;

SerialPortConfig* SerialPortConfig::New(::google::protobuf::Arena* arena) const {
  SerialPortConfig* n = new SerialPortConfig;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void SerialPortConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:roborts_serial.SerialPortConfig)
#if defined(__clang__)
#define ZR_HELPER_(f) \
  _Pragma("clang diagnostic push") \
  _Pragma("clang diagnostic ignored \"-Winvalid-offsetof\"") \
  __builtin_offsetof(SerialPortConfig, f) \
  _Pragma("clang diagnostic pop")
#else
#define ZR_HELPER_(f) reinterpret_cast<char*>(\
  &reinterpret_cast<SerialPortConfig*>(16)->f)
#endif

#define ZR_(first, last) do {\
  ::memset(&first, 0,\
           ZR_HELPER_(last) - ZR_HELPER_(first) + sizeof(last));\
} while (0)

  if (_has_bits_[0 / 32] & 255u) {
    ZR_(serial_boudrate_, flow_control_);
    if (has_serial_port()) {
      serial_port_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    }
    databits_ = 8;
    stopbits_ = 1;
    if (has_parity()) {
      parity_.ClearToDefaultNoArena(_default_parity_);
    }
    is_debug_ = true;
    is_simulator_ = false;
  }
  if (_has_bits_[8 / 32] & 768u) {
    link_column_ = 50;
    link_beam_ = 90;
  }

#undef ZR_HELPER_
#undef ZR_

  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  if (_internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->Clear();
  }
}

bool SerialPortConfig::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:roborts_serial.SerialPortConfig)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required string serial_port = 1;
      case 1: {
        if (tag == 10) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_serial_port()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->serial_port().data(), this->serial_port().length(),
            ::google::protobuf::internal::WireFormat::PARSE,
            "roborts_serial.SerialPortConfig.serial_port");
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(16)) goto parse_serial_boudrate;
        break;
      }

      // required int32 serial_boudrate = 2;
      case 2: {
        if (tag == 16) {
         parse_serial_boudrate:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &serial_boudrate_)));
          set_has_serial_boudrate();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(24)) goto parse_flow_control;
        break;
      }

      // optional int32 flow_control = 3 [default = 0];
      case 3: {
        if (tag == 24) {
         parse_flow_control:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &flow_control_)));
          set_has_flow_control();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(32)) goto parse_databits;
        break;
      }

      // optional int32 databits = 4 [default = 8];
      case 4: {
        if (tag == 32) {
         parse_databits:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &databits_)));
          set_has_databits();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(40)) goto parse_stopbits;
        break;
      }

      // optional int32 stopbits = 5 [default = 1];
      case 5: {
        if (tag == 40) {
         parse_stopbits:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &stopbits_)));
          set_has_stopbits();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(50)) goto parse_parity;
        break;
      }

      // optional string parity = 6 [default = "N"];
      case 6: {
        if (tag == 50) {
         parse_parity:
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_parity()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->parity().data(), this->parity().length(),
            ::google::protobuf::internal::WireFormat::PARSE,
            "roborts_serial.SerialPortConfig.parity");
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(56)) goto parse_is_debug;
        break;
      }

      // optional bool is_debug = 7 [default = true];
      case 7: {
        if (tag == 56) {
         parse_is_debug:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &is_debug_)));
          set_has_is_debug();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(64)) goto parse_is_simulator;
        break;
      }

      // optional bool is_simulator = 8 [default = false];
      case 8: {
        if (tag == 64) {
         parse_is_simulator:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &is_simulator_)));
          set_has_is_simulator();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(73)) goto parse_link_column;
        break;
      }

      // optional double link_column = 9 [default = 50];
      case 9: {
        if (tag == 73) {
         parse_link_column:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &link_column_)));
          set_has_link_column();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(81)) goto parse_link_beam;
        break;
      }

      // optional double link_beam = 10 [default = 90];
      case 10: {
        if (tag == 81) {
         parse_link_beam:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &link_beam_)));
          set_has_link_beam();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:roborts_serial.SerialPortConfig)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:roborts_serial.SerialPortConfig)
  return false;
#undef DO_
}

void SerialPortConfig::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:roborts_serial.SerialPortConfig)
  // required string serial_port = 1;
  if (has_serial_port()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->serial_port().data(), this->serial_port().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "roborts_serial.SerialPortConfig.serial_port");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      1, this->serial_port(), output);
  }

  // required int32 serial_boudrate = 2;
  if (has_serial_boudrate()) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(2, this->serial_boudrate(), output);
  }

  // optional int32 flow_control = 3 [default = 0];
  if (has_flow_control()) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(3, this->flow_control(), output);
  }

  // optional int32 databits = 4 [default = 8];
  if (has_databits()) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(4, this->databits(), output);
  }

  // optional int32 stopbits = 5 [default = 1];
  if (has_stopbits()) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(5, this->stopbits(), output);
  }

  // optional string parity = 6 [default = "N"];
  if (has_parity()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->parity().data(), this->parity().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "roborts_serial.SerialPortConfig.parity");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      6, this->parity(), output);
  }

  // optional bool is_debug = 7 [default = true];
  if (has_is_debug()) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(7, this->is_debug(), output);
  }

  // optional bool is_simulator = 8 [default = false];
  if (has_is_simulator()) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(8, this->is_simulator(), output);
  }

  // optional double link_column = 9 [default = 50];
  if (has_link_column()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(9, this->link_column(), output);
  }

  // optional double link_beam = 10 [default = 90];
  if (has_link_beam()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(10, this->link_beam(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:roborts_serial.SerialPortConfig)
}

::google::protobuf::uint8* SerialPortConfig::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:roborts_serial.SerialPortConfig)
  // required string serial_port = 1;
  if (has_serial_port()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->serial_port().data(), this->serial_port().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "roborts_serial.SerialPortConfig.serial_port");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->serial_port(), target);
  }

  // required int32 serial_boudrate = 2;
  if (has_serial_boudrate()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(2, this->serial_boudrate(), target);
  }

  // optional int32 flow_control = 3 [default = 0];
  if (has_flow_control()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(3, this->flow_control(), target);
  }

  // optional int32 databits = 4 [default = 8];
  if (has_databits()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(4, this->databits(), target);
  }

  // optional int32 stopbits = 5 [default = 1];
  if (has_stopbits()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(5, this->stopbits(), target);
  }

  // optional string parity = 6 [default = "N"];
  if (has_parity()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->parity().data(), this->parity().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "roborts_serial.SerialPortConfig.parity");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        6, this->parity(), target);
  }

  // optional bool is_debug = 7 [default = true];
  if (has_is_debug()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(7, this->is_debug(), target);
  }

  // optional bool is_simulator = 8 [default = false];
  if (has_is_simulator()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(8, this->is_simulator(), target);
  }

  // optional double link_column = 9 [default = 50];
  if (has_link_column()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(9, this->link_column(), target);
  }

  // optional double link_beam = 10 [default = 90];
  if (has_link_beam()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(10, this->link_beam(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:roborts_serial.SerialPortConfig)
  return target;
}

int SerialPortConfig::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:roborts_serial.SerialPortConfig)
  int total_size = 0;

  if (has_serial_port()) {
    // required string serial_port = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->serial_port());
  }

  if (has_serial_boudrate()) {
    // required int32 serial_boudrate = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->serial_boudrate());
  }

  return total_size;
}
int SerialPortConfig::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:roborts_serial.SerialPortConfig)
  int total_size = 0;

  if (((_has_bits_[0] & 0x00000003) ^ 0x00000003) == 0) {  // All required fields are present.
    // required string serial_port = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->serial_port());

    // required int32 serial_boudrate = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->serial_boudrate());

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  if (_has_bits_[2 / 32] & 252u) {
    // optional int32 flow_control = 3 [default = 0];
    if (has_flow_control()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->flow_control());
    }

    // optional int32 databits = 4 [default = 8];
    if (has_databits()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->databits());
    }

    // optional int32 stopbits = 5 [default = 1];
    if (has_stopbits()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->stopbits());
    }

    // optional string parity = 6 [default = "N"];
    if (has_parity()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->parity());
    }

    // optional bool is_debug = 7 [default = true];
    if (has_is_debug()) {
      total_size += 1 + 1;
    }

    // optional bool is_simulator = 8 [default = false];
    if (has_is_simulator()) {
      total_size += 1 + 1;
    }

  }
  if (_has_bits_[8 / 32] & 768u) {
    // optional double link_column = 9 [default = 50];
    if (has_link_column()) {
      total_size += 1 + 8;
    }

    // optional double link_beam = 10 [default = 90];
    if (has_link_beam()) {
      total_size += 1 + 8;
    }

  }
  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void SerialPortConfig::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:roborts_serial.SerialPortConfig)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  const SerialPortConfig* source = 
      ::google::protobuf::internal::DynamicCastToGenerated<const SerialPortConfig>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:roborts_serial.SerialPortConfig)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:roborts_serial.SerialPortConfig)
    MergeFrom(*source);
  }
}

void SerialPortConfig::MergeFrom(const SerialPortConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:roborts_serial.SerialPortConfig)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_serial_port()) {
      set_has_serial_port();
      serial_port_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.serial_port_);
    }
    if (from.has_serial_boudrate()) {
      set_serial_boudrate(from.serial_boudrate());
    }
    if (from.has_flow_control()) {
      set_flow_control(from.flow_control());
    }
    if (from.has_databits()) {
      set_databits(from.databits());
    }
    if (from.has_stopbits()) {
      set_stopbits(from.stopbits());
    }
    if (from.has_parity()) {
      set_has_parity();
      parity_.AssignWithDefault(_default_parity_, from.parity_);
    }
    if (from.has_is_debug()) {
      set_is_debug(from.is_debug());
    }
    if (from.has_is_simulator()) {
      set_is_simulator(from.is_simulator());
    }
  }
  if (from._has_bits_[8 / 32] & (0xffu << (8 % 32))) {
    if (from.has_link_column()) {
      set_link_column(from.link_column());
    }
    if (from.has_link_beam()) {
      set_link_beam(from.link_beam());
    }
  }
  if (from._internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->MergeFrom(from.unknown_fields());
  }
}

void SerialPortConfig::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:roborts_serial.SerialPortConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SerialPortConfig::CopyFrom(const SerialPortConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:roborts_serial.SerialPortConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SerialPortConfig::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000003) != 0x00000003) return false;

  return true;
}

void SerialPortConfig::Swap(SerialPortConfig* other) {
  if (other == this) return;
  InternalSwap(other);
}
void SerialPortConfig::InternalSwap(SerialPortConfig* other) {
  serial_port_.Swap(&other->serial_port_);
  std::swap(serial_boudrate_, other->serial_boudrate_);
  std::swap(flow_control_, other->flow_control_);
  std::swap(databits_, other->databits_);
  std::swap(stopbits_, other->stopbits_);
  parity_.Swap(&other->parity_);
  std::swap(is_debug_, other->is_debug_);
  std::swap(is_simulator_, other->is_simulator_);
  std::swap(link_column_, other->link_column_);
  std::swap(link_beam_, other->link_beam_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata SerialPortConfig::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = SerialPortConfig_descriptor_;
  metadata.reflection = SerialPortConfig_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// SerialPortConfig

// required string serial_port = 1;
bool SerialPortConfig::has_serial_port() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void SerialPortConfig::set_has_serial_port() {
  _has_bits_[0] |= 0x00000001u;
}
void SerialPortConfig::clear_has_serial_port() {
  _has_bits_[0] &= ~0x00000001u;
}
void SerialPortConfig::clear_serial_port() {
  serial_port_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_serial_port();
}
 const ::std::string& SerialPortConfig::serial_port() const {
  // @@protoc_insertion_point(field_get:roborts_serial.SerialPortConfig.serial_port)
  return serial_port_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void SerialPortConfig::set_serial_port(const ::std::string& value) {
  set_has_serial_port();
  serial_port_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:roborts_serial.SerialPortConfig.serial_port)
}
 void SerialPortConfig::set_serial_port(const char* value) {
  set_has_serial_port();
  serial_port_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:roborts_serial.SerialPortConfig.serial_port)
}
 void SerialPortConfig::set_serial_port(const char* value, size_t size) {
  set_has_serial_port();
  serial_port_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:roborts_serial.SerialPortConfig.serial_port)
}
 ::std::string* SerialPortConfig::mutable_serial_port() {
  set_has_serial_port();
  // @@protoc_insertion_point(field_mutable:roborts_serial.SerialPortConfig.serial_port)
  return serial_port_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 ::std::string* SerialPortConfig::release_serial_port() {
  // @@protoc_insertion_point(field_release:roborts_serial.SerialPortConfig.serial_port)
  clear_has_serial_port();
  return serial_port_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void SerialPortConfig::set_allocated_serial_port(::std::string* serial_port) {
  if (serial_port != NULL) {
    set_has_serial_port();
  } else {
    clear_has_serial_port();
  }
  serial_port_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), serial_port);
  // @@protoc_insertion_point(field_set_allocated:roborts_serial.SerialPortConfig.serial_port)
}

// required int32 serial_boudrate = 2;
bool SerialPortConfig::has_serial_boudrate() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void SerialPortConfig::set_has_serial_boudrate() {
  _has_bits_[0] |= 0x00000002u;
}
void SerialPortConfig::clear_has_serial_boudrate() {
  _has_bits_[0] &= ~0x00000002u;
}
void SerialPortConfig::clear_serial_boudrate() {
  serial_boudrate_ = 0;
  clear_has_serial_boudrate();
}
 ::google::protobuf::int32 SerialPortConfig::serial_boudrate() const {
  // @@protoc_insertion_point(field_get:roborts_serial.SerialPortConfig.serial_boudrate)
  return serial_boudrate_;
}
 void SerialPortConfig::set_serial_boudrate(::google::protobuf::int32 value) {
  set_has_serial_boudrate();
  serial_boudrate_ = value;
  // @@protoc_insertion_point(field_set:roborts_serial.SerialPortConfig.serial_boudrate)
}

// optional int32 flow_control = 3 [default = 0];
bool SerialPortConfig::has_flow_control() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
void SerialPortConfig::set_has_flow_control() {
  _has_bits_[0] |= 0x00000004u;
}
void SerialPortConfig::clear_has_flow_control() {
  _has_bits_[0] &= ~0x00000004u;
}
void SerialPortConfig::clear_flow_control() {
  flow_control_ = 0;
  clear_has_flow_control();
}
 ::google::protobuf::int32 SerialPortConfig::flow_control() const {
  // @@protoc_insertion_point(field_get:roborts_serial.SerialPortConfig.flow_control)
  return flow_control_;
}
 void SerialPortConfig::set_flow_control(::google::protobuf::int32 value) {
  set_has_flow_control();
  flow_control_ = value;
  // @@protoc_insertion_point(field_set:roborts_serial.SerialPortConfig.flow_control)
}

// optional int32 databits = 4 [default = 8];
bool SerialPortConfig::has_databits() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
void SerialPortConfig::set_has_databits() {
  _has_bits_[0] |= 0x00000008u;
}
void SerialPortConfig::clear_has_databits() {
  _has_bits_[0] &= ~0x00000008u;
}
void SerialPortConfig::clear_databits() {
  databits_ = 8;
  clear_has_databits();
}
 ::google::protobuf::int32 SerialPortConfig::databits() const {
  // @@protoc_insertion_point(field_get:roborts_serial.SerialPortConfig.databits)
  return databits_;
}
 void SerialPortConfig::set_databits(::google::protobuf::int32 value) {
  set_has_databits();
  databits_ = value;
  // @@protoc_insertion_point(field_set:roborts_serial.SerialPortConfig.databits)
}

// optional int32 stopbits = 5 [default = 1];
bool SerialPortConfig::has_stopbits() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
void SerialPortConfig::set_has_stopbits() {
  _has_bits_[0] |= 0x00000010u;
}
void SerialPortConfig::clear_has_stopbits() {
  _has_bits_[0] &= ~0x00000010u;
}
void SerialPortConfig::clear_stopbits() {
  stopbits_ = 1;
  clear_has_stopbits();
}
 ::google::protobuf::int32 SerialPortConfig::stopbits() const {
  // @@protoc_insertion_point(field_get:roborts_serial.SerialPortConfig.stopbits)
  return stopbits_;
}
 void SerialPortConfig::set_stopbits(::google::protobuf::int32 value) {
  set_has_stopbits();
  stopbits_ = value;
  // @@protoc_insertion_point(field_set:roborts_serial.SerialPortConfig.stopbits)
}

// optional string parity = 6 [default = "N"];
bool SerialPortConfig::has_parity() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
void SerialPortConfig::set_has_parity() {
  _has_bits_[0] |= 0x00000020u;
}
void SerialPortConfig::clear_has_parity() {
  _has_bits_[0] &= ~0x00000020u;
}
void SerialPortConfig::clear_parity() {
  parity_.ClearToDefaultNoArena(_default_parity_);
  clear_has_parity();
}
 const ::std::string& SerialPortConfig::parity() const {
  // @@protoc_insertion_point(field_get:roborts_serial.SerialPortConfig.parity)
  return parity_.GetNoArena(_default_parity_);
}
 void SerialPortConfig::set_parity(const ::std::string& value) {
  set_has_parity();
  parity_.SetNoArena(_default_parity_, value);
  // @@protoc_insertion_point(field_set:roborts_serial.SerialPortConfig.parity)
}
 void SerialPortConfig::set_parity(const char* value) {
  set_has_parity();
  parity_.SetNoArena(_default_parity_, ::std::string(value));
  // @@protoc_insertion_point(field_set_char:roborts_serial.SerialPortConfig.parity)
}
 void SerialPortConfig::set_parity(const char* value, size_t size) {
  set_has_parity();
  parity_.SetNoArena(_default_parity_,
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:roborts_serial.SerialPortConfig.parity)
}
 ::std::string* SerialPortConfig::mutable_parity() {
  set_has_parity();
  // @@protoc_insertion_point(field_mutable:roborts_serial.SerialPortConfig.parity)
  return parity_.MutableNoArena(_default_parity_);
}
 ::std::string* SerialPortConfig::release_parity() {
  // @@protoc_insertion_point(field_release:roborts_serial.SerialPortConfig.parity)
  clear_has_parity();
  return parity_.ReleaseNoArena(_default_parity_);
}
 void SerialPortConfig::set_allocated_parity(::std::string* parity) {
  if (parity != NULL) {
    set_has_parity();
  } else {
    clear_has_parity();
  }
  parity_.SetAllocatedNoArena(_default_parity_, parity);
  // @@protoc_insertion_point(field_set_allocated:roborts_serial.SerialPortConfig.parity)
}

// optional bool is_debug = 7 [default = true];
bool SerialPortConfig::has_is_debug() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
void SerialPortConfig::set_has_is_debug() {
  _has_bits_[0] |= 0x00000040u;
}
void SerialPortConfig::clear_has_is_debug() {
  _has_bits_[0] &= ~0x00000040u;
}
void SerialPortConfig::clear_is_debug() {
  is_debug_ = true;
  clear_has_is_debug();
}
 bool SerialPortConfig::is_debug() const {
  // @@protoc_insertion_point(field_get:roborts_serial.SerialPortConfig.is_debug)
  return is_debug_;
}
 void SerialPortConfig::set_is_debug(bool value) {
  set_has_is_debug();
  is_debug_ = value;
  // @@protoc_insertion_point(field_set:roborts_serial.SerialPortConfig.is_debug)
}

// optional bool is_simulator = 8 [default = false];
bool SerialPortConfig::has_is_simulator() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
void SerialPortConfig::set_has_is_simulator() {
  _has_bits_[0] |= 0x00000080u;
}
void SerialPortConfig::clear_has_is_simulator() {
  _has_bits_[0] &= ~0x00000080u;
}
void SerialPortConfig::clear_is_simulator() {
  is_simulator_ = false;
  clear_has_is_simulator();
}
 bool SerialPortConfig::is_simulator() const {
  // @@protoc_insertion_point(field_get:roborts_serial.SerialPortConfig.is_simulator)
  return is_simulator_;
}
 void SerialPortConfig::set_is_simulator(bool value) {
  set_has_is_simulator();
  is_simulator_ = value;
  // @@protoc_insertion_point(field_set:roborts_serial.SerialPortConfig.is_simulator)
}

// optional double link_column = 9 [default = 50];
bool SerialPortConfig::has_link_column() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
void SerialPortConfig::set_has_link_column() {
  _has_bits_[0] |= 0x00000100u;
}
void SerialPortConfig::clear_has_link_column() {
  _has_bits_[0] &= ~0x00000100u;
}
void SerialPortConfig::clear_link_column() {
  link_column_ = 50;
  clear_has_link_column();
}
 double SerialPortConfig::link_column() const {
  // @@protoc_insertion_point(field_get:roborts_serial.SerialPortConfig.link_column)
  return link_column_;
}
 void SerialPortConfig::set_link_column(double value) {
  set_has_link_column();
  link_column_ = value;
  // @@protoc_insertion_point(field_set:roborts_serial.SerialPortConfig.link_column)
}

// optional double link_beam = 10 [default = 90];
bool SerialPortConfig::has_link_beam() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
void SerialPortConfig::set_has_link_beam() {
  _has_bits_[0] |= 0x00000200u;
}
void SerialPortConfig::clear_has_link_beam() {
  _has_bits_[0] &= ~0x00000200u;
}
void SerialPortConfig::clear_link_beam() {
  link_beam_ = 90;
  clear_has_link_beam();
}
 double SerialPortConfig::link_beam() const {
  // @@protoc_insertion_point(field_get:roborts_serial.SerialPortConfig.link_beam)
  return link_beam_;
}
 void SerialPortConfig::set_link_beam(double value) {
  set_has_link_beam();
  link_beam_ = value;
  // @@protoc_insertion_point(field_set:roborts_serial.SerialPortConfig.link_beam)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace roborts_serial

// @@protoc_insertion_point(global_scope)
