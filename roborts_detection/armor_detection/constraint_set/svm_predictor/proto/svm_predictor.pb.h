// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: svm_predictor.proto

#ifndef PROTOBUF_svm_5fpredictor_2eproto__INCLUDED
#define PROTOBUF_svm_5fpredictor_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3000000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3000000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_svm_5fpredictor_2eproto();
void protobuf_AssignDesc_svm_5fpredictor_2eproto();
void protobuf_ShutdownFile_svm_5fpredictor_2eproto();

class SVMPredictorConfig;

// ===================================================================

class SVMPredictorConfig : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:SVMPredictorConfig) */ {
 public:
  SVMPredictorConfig();
  virtual ~SVMPredictorConfig();

  SVMPredictorConfig(const SVMPredictorConfig& from);

  inline SVMPredictorConfig& operator=(const SVMPredictorConfig& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const SVMPredictorConfig& default_instance();

  void Swap(SVMPredictorConfig* other);

  // implements Message ----------------------------------------------

  inline SVMPredictorConfig* New() const { return New(NULL); }

  SVMPredictorConfig* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const SVMPredictorConfig& from);
  void MergeFrom(const SVMPredictorConfig& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(SVMPredictorConfig* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required string model_name = 1;
  bool has_model_name() const;
  void clear_model_name();
  static const int kModelNameFieldNumber = 1;
  const ::std::string& model_name() const;
  void set_model_name(const ::std::string& value);
  void set_model_name(const char* value);
  void set_model_name(const char* value, size_t size);
  ::std::string* mutable_model_name();
  ::std::string* release_model_name();
  void set_allocated_model_name(::std::string* model_name);

  // required int32 resize_height = 2;
  bool has_resize_height() const;
  void clear_resize_height();
  static const int kResizeHeightFieldNumber = 2;
  ::google::protobuf::int32 resize_height() const;
  void set_resize_height(::google::protobuf::int32 value);

  // required int32 resize_width = 3;
  bool has_resize_width() const;
  void clear_resize_width();
  static const int kResizeWidthFieldNumber = 3;
  ::google::protobuf::int32 resize_width() const;
  void set_resize_width(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:SVMPredictorConfig)
 private:
  inline void set_has_model_name();
  inline void clear_has_model_name();
  inline void set_has_resize_height();
  inline void clear_has_resize_height();
  inline void set_has_resize_width();
  inline void clear_has_resize_width();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr model_name_;
  ::google::protobuf::int32 resize_height_;
  ::google::protobuf::int32 resize_width_;
  friend void  protobuf_AddDesc_svm_5fpredictor_2eproto();
  friend void protobuf_AssignDesc_svm_5fpredictor_2eproto();
  friend void protobuf_ShutdownFile_svm_5fpredictor_2eproto();

  void InitAsDefaultInstance();
  static SVMPredictorConfig* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// SVMPredictorConfig

// required string model_name = 1;
inline bool SVMPredictorConfig::has_model_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void SVMPredictorConfig::set_has_model_name() {
  _has_bits_[0] |= 0x00000001u;
}
inline void SVMPredictorConfig::clear_has_model_name() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void SVMPredictorConfig::clear_model_name() {
  model_name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_model_name();
}
inline const ::std::string& SVMPredictorConfig::model_name() const {
  // @@protoc_insertion_point(field_get:SVMPredictorConfig.model_name)
  return model_name_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void SVMPredictorConfig::set_model_name(const ::std::string& value) {
  set_has_model_name();
  model_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:SVMPredictorConfig.model_name)
}
inline void SVMPredictorConfig::set_model_name(const char* value) {
  set_has_model_name();
  model_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:SVMPredictorConfig.model_name)
}
inline void SVMPredictorConfig::set_model_name(const char* value, size_t size) {
  set_has_model_name();
  model_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:SVMPredictorConfig.model_name)
}
inline ::std::string* SVMPredictorConfig::mutable_model_name() {
  set_has_model_name();
  // @@protoc_insertion_point(field_mutable:SVMPredictorConfig.model_name)
  return model_name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* SVMPredictorConfig::release_model_name() {
  // @@protoc_insertion_point(field_release:SVMPredictorConfig.model_name)
  clear_has_model_name();
  return model_name_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void SVMPredictorConfig::set_allocated_model_name(::std::string* model_name) {
  if (model_name != NULL) {
    set_has_model_name();
  } else {
    clear_has_model_name();
  }
  model_name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), model_name);
  // @@protoc_insertion_point(field_set_allocated:SVMPredictorConfig.model_name)
}

// required int32 resize_height = 2;
inline bool SVMPredictorConfig::has_resize_height() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void SVMPredictorConfig::set_has_resize_height() {
  _has_bits_[0] |= 0x00000002u;
}
inline void SVMPredictorConfig::clear_has_resize_height() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void SVMPredictorConfig::clear_resize_height() {
  resize_height_ = 0;
  clear_has_resize_height();
}
inline ::google::protobuf::int32 SVMPredictorConfig::resize_height() const {
  // @@protoc_insertion_point(field_get:SVMPredictorConfig.resize_height)
  return resize_height_;
}
inline void SVMPredictorConfig::set_resize_height(::google::protobuf::int32 value) {
  set_has_resize_height();
  resize_height_ = value;
  // @@protoc_insertion_point(field_set:SVMPredictorConfig.resize_height)
}

// required int32 resize_width = 3;
inline bool SVMPredictorConfig::has_resize_width() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void SVMPredictorConfig::set_has_resize_width() {
  _has_bits_[0] |= 0x00000004u;
}
inline void SVMPredictorConfig::clear_has_resize_width() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void SVMPredictorConfig::clear_resize_width() {
  resize_width_ = 0;
  clear_has_resize_width();
}
inline ::google::protobuf::int32 SVMPredictorConfig::resize_width() const {
  // @@protoc_insertion_point(field_get:SVMPredictorConfig.resize_width)
  return resize_width_;
}
inline void SVMPredictorConfig::set_resize_width(::google::protobuf::int32 value) {
  set_has_resize_width();
  resize_width_ = value;
  // @@protoc_insertion_point(field_set:SVMPredictorConfig.resize_width)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_svm_5fpredictor_2eproto__INCLUDED
