#!/bin/bash

# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# This script creates default user and bypass the oem configuration setup

set -e

function usage()
{
	echo "Usage:"
	echo "${script_name} [-u <username>] [-p <password>] [-a] [-h]"
	echo "	-u | --username	- If not specified then default will be set to 'nvidia'."
	echo "	-p | --password	- If not set then randomized password will be generated."
	echo "	-a | --autologin - If specified autologin will be enabled. Default is disabled"
	echo "	-h | --help - print usage"
	echo "Example:"
	echo "${script_name} -u nvidia -p NDZjMWM4"
	echo "${script_name} -u ubuntu -a"
	exit 1
}

function cleanup() {
	set +e
	pushd "${rfs_dir}" > /dev/null 2>&1

	for attempt in $(seq 10); do
		mount | grep -q "${rfs_dir}/sys" && umount ./sys
		mount | grep -q "${rfs_dir}/proc" && umount ./proc
		mount | grep -q "${rfs_dir}/dev" && umount ./dev
		mount | grep -q "${rfs_dir}"
		if [ $? -ne 0 ]; then
			break
		fi
		sleep 1
	done
	popd > /dev/null
}
trap cleanup EXIT

function generate_randomized_password()
{
	user_pass="$(date +%s | sha256sum | base64 | head -c 8)"
}

function parse_args()
{
	while [ -n "${1}" ]; do
		case "${1}" in
		-h | --help)
			usage
			;;
		-a | --autologin)
			auto_login=true
			shift 1
			;;
		-u | --username)
			[ -n "${2}" ] || usage || echo "ERROR: Not enough parameters"
			user_name="${2}"
			shift 2
			;;
		-p | --password)
			[ -n "${2}" ] || usage || echo "ERROR: Not enough parameters"
			user_pass="${2}"
			shift 2
			;;
		*)
			echo "ERROR: Invalid parameter. Exiting..."
			usage
			exit 1
			;;
		esac
	done
}

function check_pre_req()
{
	this_user="$(whoami)"
	if [ "${this_user}" != "root" ]; then
		echo "ERROR: please run as sudo or root user" > /dev/stderr
		usage
		exit 1
	fi

	if [ ! -d "${rfs_dir}" ]; then
		echo "ERROR: ${rfs_dir} directory not found" > /dev/stderr
		usage
	fi

	if [ ! -f "${rfs_dir}/etc/passwd" ]; then
		echo "ERROR: not a valid rootfs directory" > /dev/stderr
		usage
	fi

	if [ ! -f "/usr/bin/qemu-aarch64-static" ]; then
		echo "ERROR: please install qemu-user-static package" > /dev/stderr
		usage
	fi

	if [ "${user_name}" == "" ]; then
		user_name="nvidia"
	fi

	if [ "${user_pass}" == "" ]; then
		generate_randomized_password
	fi
}

function create_autologin()
{
	if [ "${auto_login}" = true ]; then
		# Set default Autologin user for GDM3 display manager.
		if [ -e "etc/gdm3/custom.conf" ]; then
			sed -i "/AutomaticLoginEnable =/ s/^.*/AutomaticLoginEnable = true/" \
				"etc/gdm3/custom.conf"
			sed -i "/AutomaticLogin =/ s/^.*/AutomaticLogin = ${user_name}/" \
				"etc/gdm3/custom.conf"
		fi

		# Set default Autologin user for debug serial console.
		mkdir -p "etc/systemd/system/serial-getty@ttyS0.service.d"
		echo "[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin ${user_name} --keep-baud 115200 %I ${TERM}" \
			> "etc/systemd/system/serial-getty@ttyS0.service.d/autologin.conf"
		mkdir -p "etc/systemd/system/serial-getty@ttyTCU0.service.d"
		echo "[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin ${user_name} --keep-baud 115200 %I ${TERM}" \
			> "etc/systemd/system/serial-getty@ttyTCU0.service.d/autologin.conf"
	else
		if [ -e "etc/gdm3/custom.conf" ]; then
			sed -i '/AutomaticLoginEnable =/ s/^/#/g' "etc/gdm3/custom.conf"
			sed -i '/AutomaticLogin =/ s/^/#/g' "etc/gdm3/custom.conf"
		fi
		rm -rf "etc/systemd/system/serial-getty@ttyS0.service.d/autologin.conf"
		rm -rf "etc/systemd/system/serial-getty@ttyTCU0.service.d/autologin.conf"
	fi
}

function create_user()
{
	echo "Creating: Username - ${user_name}, Password - ${user_pass}, Autologin - ${auto_login}"
	pushd "${rfs_dir}" > /dev/null 2>&1
	cp "/usr/bin/qemu-aarch64-static" "usr/bin/"
	chmod 755 "usr/bin/qemu-aarch64-static"
	mount /sys ./sys -o bind
	mount /proc ./proc -o bind
	mount /dev ./dev -o bind

	if [ -d "${rfs_dir}/home/${user_name}" ]; then
		LC_ALL=C chroot . useradd -G "sudo,video,audio,adm" -s "/bin/bash" \
			-p "$(echo "${user_pass}" | openssl passwd -1 -stdin)" "${user_name}"
		cp -r "${rfs_dir}/etc/skel/." "${rfs_dir}/home/${user_name}"
		chown -v -R `cat ${rfs_dir}/etc/passwd | grep ${user_name} | cut -d : -f 3-4` \
			"${rfs_dir}/home/${user_name}" > /dev/null
	else
		LC_ALL=C chroot . useradd -d "/home/${user_name}" -m \
			-G "sudo,video,audio,adm" -s "/bin/bash" \
			-p "$(echo "${user_pass}" | openssl passwd -1 -stdin)" "${user_name}"
	fi

	if [ -e "etc/gdm3/custom.conf" ]; then
		LC_ALL=C chroot . addgroup "${user_name}" "gdm"
		LC_ALL=C chroot . addgroup "gdm" "video"
	fi

	# enable/disable autologin
	create_autologin

	# remove default.target symlink to bypass oem config setup
	rm -f etc/systemd/system/default.target
	rm -f usr/bin/qemu-aarch64-static

	popd > /dev/null
}

script_name="$(basename "${0}")"
l4t_dir="$(cd "$(dirname "${0}")" && pwd)"
rfs_dir="${l4t_dir}/rootfs"
tmpdir=""
user_name=""
user_pass=""
auto_login=false

parse_args "${@}"
check_pre_req
create_user
