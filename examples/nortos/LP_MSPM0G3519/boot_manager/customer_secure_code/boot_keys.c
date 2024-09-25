/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <bootutil/sign_key.h>

#include <mcuboot_config/mcuboot_config.h>

/* The following is an ECDSA p256 public key in DER format that should be
 * generated from the same key that will be used to sign the device.
 */

/* clang-format off */

/* Autogenerated by imgtool.py, do not edit. */
const unsigned char ecdsa_pub_key[] = {
    0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2a, 0x86,
    0x48, 0xce, 0x3d, 0x02, 0x01, 0x06, 0x08, 0x2a,
    0x86, 0x48, 0xce, 0x3d, 0x03, 0x01, 0x07, 0x03,
    0x42, 0x00, 0x04, 0x41, 0xcc, 0x66, 0xc6, 0x28,
    0xa2, 0xa5, 0x95, 0x4e, 0x60, 0xc2, 0x03, 0xe0,
    0x3b, 0x81, 0xa1, 0x09, 0xa6, 0x2b, 0x37, 0x22,
    0x53, 0x46, 0xed, 0xd0, 0x0a, 0x76, 0x6e, 0x06,
    0x79, 0x3a, 0x35, 0xd8, 0xaa, 0x7f, 0x11, 0x79,
    0xa8, 0xb0, 0xba, 0x4d, 0x44, 0xc8, 0x45, 0x0b,
    0x48, 0x48, 0x23, 0x64, 0x97, 0x5a, 0xdd, 0xeb,
    0x3c, 0x5b, 0x5d, 0xee, 0xb2, 0x73, 0x80, 0x80,
    0xc5, 0xbb, 0xf1,
};
const unsigned int ecdsa_pub_key_len = 91;

/* clang-format on */

const struct bootutil_key bootutil_keys[] = {
    {
        .key = ecdsa_pub_key,
        .len = &ecdsa_pub_key_len,
    },
};
const int bootutil_key_cnt = 1;
