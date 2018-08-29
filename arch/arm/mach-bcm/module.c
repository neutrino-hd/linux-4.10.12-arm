/*
 * Copyright (C) 2009 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/moduleloader.h>
#include <linux/elf.h>
#include <linux/kernel.h>
#include <linux/module.h>


int (*module_frob_sections)(Elf_Ehdr *hdr, Elf_Shdr *sechdrs, char *secstrings, struct module *mod);
EXPORT_SYMBOL(module_frob_sections);

int module_frob_arch_sections(Elf_Ehdr *hdr, Elf_Shdr *sechdrs, char *secstrings, struct module *mod)
{
	if (module_frob_sections) {
		int err = module_frob_sections(hdr, sechdrs, secstrings, mod);
		if (err < 0)
			return err;
	}
	return 0;
}
