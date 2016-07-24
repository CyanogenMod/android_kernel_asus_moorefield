#ifndef __IA_CSS_ISYS_STREAM_DEVICE_PRIVATE_H_INCLUDED__
#define __IA_CSS_ISYS_STREAM_DEVICE_PRIVATE_H_INCLUDED__

/* Enums, structures, unions that are used within the API functions */


/* Stream2MMIO.  BXT has 4(?)*/
enum stream2mmio_ID_t{
	STREAM2MMIO0_ID = 0,	/* map to ISYS2600_S2M_A */
	STREAM2MMIO1_ID,	/* map to ISYS2600_S2M_B */
	STREAM2MMIO2_ID,	/* map to ISYS2600_S2M_C */
	STREAM2MMIO3_ID,	/* map to ISYS2600_S2M_D */
	N_STREAM2MMIO_ID
};

 /* Each Stream2MMIO has 4 SIDs (?) which are indexed by
  * [STREAM2MMIO_SID0_ID...TREAM2MMIO_SID3_ID].
  */
enum stream2mmio_sid_ID_t{
	STREAM2MMIO_SID0_ID = 0,
	STREAM2MMIO_SID1_ID,
	STREAM2MMIO_SID2_ID,
	STREAM2MMIO_SID3_ID,
	N_STREAM2MMIO_SID_ID
};

 /* Input-buffer Controller. BXT has 2 (?) */
enum ibuf_ctrl_ID_t{
	IBUF_CTRL0_ID = 0,	/* map to ISYS2600_IBUF_CNTRL_A */
	IBUF_CTRL1_ID,		/* map to ISYS2600_IBUF_CNTRL_B */
	N_IBUF_CTRL_ID
};

 /* DMA2600. BXT has 1(?) which has 11 channels (?) */
enum isys2600_dma_ID_t{
	ISYS2600_DMA0_ID = 0,
	N_ISYS2600_DMA_ID
};

enum ISYS2600_DMA_CHANNEL{
	ISYS2600_DMA_CHANNEL_0 = 0,
	ISYS2600_DMA_CHANNEL_1,
	ISYS2600_DMA_CHANNEL_2,
	ISYS2600_DMA_CHANNEL_3,
	ISYS2600_DMA_CHANNEL_4,
	ISYS2600_DMA_CHANNEL_5,
	ISYS2600_DMA_CHANNEL_6,
	ISYS2600_DMA_CHANNEL_7,
	ISYS2600_DMA_CHANNEL_8,
	ISYS2600_DMA_CHANNEL_9,
	ISYS2600_DMA_CHANNEL_10,
	ISYS2600_DMA_CHANNEL_11,
	N_ISYS2600_DMA_CHANNEL
};

struct	ib_buffer_t {
	unsigned int start_addr;	/* start address of the buffer */
	unsigned int stride;		/* stride per buffer line (in bytes) */
	unsigned int lines;		/* lines in the buffer */
};

/* This is the identifier structure for each of the stream device groups
 * It is filled in by the stream controller layer, which takes as input parameters the
 * dev_cfg data that the driver sends through the communication layer(queues)
 */
struct istream_device {
	enum stream2mmio_ID_t stream2mmio_id;
	enum stream2mmio_sid_ID_t	stream2mmio_sid_id;
	enum ibuf_ctrl_ID_t		    ibuf_ctrl_id;
	struct ib_buffer_t		        ib_buffer;
	enum isys2600_dma_ID_t	    dma_id;
	enum ISYS2600_DMA_CHANNEL	dma_channel;
};

 /* The NCIs for each HW block(stream2mmio, dma2600, ibuf_ctrl)
  * provided by the HW team will include the declaration of the _t structures
  */
struct stream2mmio_cfg_t {
	unsigned int dummy;
};

struct ibuf_ctrl_cfg_t {
	unsigned int dummy;
};

struct isys2600_dma_cfg_t {
	unsigned int dummy;
};

struct isys2600_dma_port_cfg_t {
	unsigned int dummy;
};
 /* This is the structure that configures the stream device group with specific
  * configuration data
  * Its members configure the HW blocks of a stream device group
  *	specified by the stream device group identifier.
  *
  */
 struct istream_device_cfg {
	struct stream2mmio_cfg_t	stream2mmio_cfg;
	struct ibuf_ctrl_cfg_t		ibuf_ctrl_cfg;
	struct isys2600_dma_cfg_t	dma_cfg;
	struct isys2600_dma_port_cfg_t	dma_src_port_cfg;
	struct isys2600_dma_port_cfg_t	dma_dest_port_cfg;
};

#endif /* __IA_CSS_ISYS_STREAM_DEVICE_PRIVATE_H_INCLUDED__*/
