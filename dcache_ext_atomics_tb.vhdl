-- Directed test for the dcache EXT_ATOMICS mode (external lwarx/stcx.
-- verdict, e.g. an OpenPiton/BYOC L1.5 behind a Wishbone bridge).
--
-- Run with EXT => true (default): every check must pass.
-- Run with EXT => false (-gEXT=false): the checks must FAIL, because the
-- memory system's verdict is then ignored - that is the unpatched behaviour.
-- DCBZ_ALLOC => false (default) drives dcache DCBZ_ALLOCATE; checks EXT8/9
-- must FAIL with -gDCBZ_ALLOC=true (a dcbz miss then allocates the line).
--
-- The memory model below is the far-side oracle: it records every accepted
-- Wishbone beat (we, adr, ext_reserve) and, for a reserved write, returns
-- ext_sc_fail together with the ack and does NOT perform a failed write.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.common.all;
use work.wishbone_types.all;

entity dcache_ext_atomics_tb is
    generic (
        EXT : boolean := true;
        DCBZ_ALLOC : boolean := false
        );
end dcache_ext_atomics_tb;

architecture behave of dcache_ext_atomics_tb is
    signal clk          : std_ulogic;
    signal rst          : std_ulogic;

    signal d_in         : Loadstore1ToDcacheType := Loadstore1ToDcacheInit;
    signal d_out        : DcacheToLoadstore1Type;
    signal m_in         : MmuToDcacheType;
    signal m_out        : DcacheToMmuType;
    signal stall        : std_ulogic;

    signal wb_out       : wishbone_master_out;
    signal wb_in        : wishbone_slave_out := wishbone_slave_out_init;
    signal snoop        : wishbone_master_out := wishbone_master_out_init;
    signal ext_reserve  : std_ulogic;
    signal ext_sc_fail  : std_ulogic := '0';

    constant clk_period : time := 10 ns;

    -- memory-side controls and observations
    signal fail_next_sc : std_ulogic := '0';     -- verdict for the next reserved write
    signal n_reads      : natural := 0;
    signal n_res_reads  : natural := 0;          -- reads accepted with ext_reserve = 1
    signal n_writes     : natural := 0;
    signal n_res_writes : natural := 0;          -- writes accepted with ext_reserve = 1

    function pattern(i : natural) return std_ulogic_vector is
        variable r : std_ulogic_vector(63 downto 0);
    begin
        r := std_ulogic_vector(to_unsigned(16#1000# + i, 32)) &
             std_ulogic_vector(to_unsigned(16#2000# + i, 32));
        return r;
    end function;
begin
    dcache0: entity work.dcache
        generic map(
            LINE_SIZE => 16,
            NUM_LINES => 4,
            NUM_WAYS => 1,
            EXT_ATOMICS => EXT,
            DCBZ_ALLOCATE => DCBZ_ALLOC
            )
        port map(
            clk => clk,
            rst => rst,
            d_in => d_in,
            d_out => d_out,
            stall_out => stall,
            m_in => m_in,
            m_out => m_out,
            snoop_in => snoop,
            wishbone_out => wb_out,
            wishbone_in => wb_in,
            ext_reserve => ext_reserve,
            ext_sc_fail => ext_sc_fail
            );

    clk_process: process
    begin
        clk <= '0';
        wait for clk_period/2;
        clk <= '1';
        wait for clk_period/2;
    end process;

    -- Pipelined Wishbone memory: stalls every third cycle, acks each accepted
    -- beat exactly one cycle later.
    mem: process(clk)
        type mem_t is array(0 to 255) of std_ulogic_vector(63 downto 0);
        variable m        : mem_t;
        variable inited   : boolean := false;
        variable p_valid  : boolean := false;
        variable p_we     : std_ulogic;
        variable p_res    : std_ulogic;
        variable p_adr    : natural;
        variable p_dat    : std_ulogic_vector(63 downto 0);
        variable p_sel    : std_ulogic_vector(7 downto 0);
        variable p_fail   : std_ulogic;
        variable cnt      : natural := 0;
    begin
        if rising_edge(clk) then
            if not inited then
                for i in m'range loop
                    m(i) := pattern(i);
                end loop;
                inited := true;
            end if;
            wb_in.ack <= '0';
            ext_sc_fail <= '0';
            -- ack the beat accepted last cycle
            if p_valid then
                wb_in.ack <= '1';
                if p_we = '1' then
                    ext_sc_fail <= p_fail;
                    if p_fail = '0' then
                        for b in 0 to 7 loop
                            if p_sel(b) = '1' then
                                m(p_adr)(b*8 + 7 downto b*8) := p_dat(b*8 + 7 downto b*8);
                            end if;
                        end loop;
                    end if;
                else
                    wb_in.dat <= m(p_adr);
                end if;
                p_valid := false;
            end if;
            -- accept a new beat (stall is the value the master saw this cycle)
            if rst = '0' and wb_out.cyc = '1' and wb_out.stb = '1' and wb_in.stall = '0' then
                p_valid := true;
                p_we := wb_out.we;
                p_res := ext_reserve;
                p_adr := to_integer(unsigned(wb_out.adr(7 downto 0)));
                p_dat := wb_out.dat;
                p_sel := wb_out.sel;
                p_fail := wb_out.we and ext_reserve and fail_next_sc;
                report "mem: accept we=" & std_ulogic'image(wb_out.we) &
                    " dw=" & integer'image(p_adr) &
                    " sel=" & to_hstring(wb_out.sel) &
                    " ext_reserve=" & std_ulogic'image(ext_reserve);
                if wb_out.we = '1' then
                    n_writes <= n_writes + 1;
                    if ext_reserve = '1' then
                        n_res_writes <= n_res_writes + 1;
                    end if;
                else
                    n_reads <= n_reads + 1;
                    if ext_reserve = '1' then
                        n_res_reads <= n_res_reads + 1;
                    end if;
                end if;
            end if;
            cnt := cnt + 1;
            if cnt mod 3 = 0 then
                wb_in.stall <= '1';
            else
                wb_in.stall <= '0';
            end if;
        end if;
    end process;

    stim: process
        variable reads0, res_reads0, writes0, res_writes0 : natural;
        variable done : std_ulogic;
        variable data : std_ulogic_vector(63 downto 0);

        procedure do_access(load, reserve : std_ulogic; addr : natural;
                          wdata : std_ulogic_vector(63 downto 0);
                          sel : std_ulogic_vector(7 downto 0)) is
        begin
            d_in.load <= load;
            d_in.reserve <= reserve;
            d_in.addr <= std_ulogic_vector(to_unsigned(addr, 64));
            d_in.data <= wdata;
            d_in.byte_sel <= sel;
            d_in.valid <= '1';
            wait until rising_edge(clk) and stall = '0';
            d_in.valid <= '0';
            wait until rising_edge(clk) and d_out.valid = '1';
            done := d_out.store_done;
            data := d_out.data;
            -- let the bus quiesce so the counters are final
            for i in 1 to 6 loop
                wait until rising_edge(clk);
            end loop;
        end procedure;

        procedure do_dcbz(addr : natural) is
        begin
            d_in.dcbz <= '1';
            do_access('0', '0', addr, (others => '0'), x"FF");
            d_in.dcbz <= '0';
        end procedure;

        procedure snapshot is
        begin
            reads0 := n_reads; res_reads0 := n_res_reads;
            writes0 := n_writes; res_writes0 := n_res_writes;
        end procedure;

        constant A      : natural := 16#100#;   -- memory dword 32
        constant STDATA : std_ulogic_vector(63 downto 0) := x"00000000CAFEF00D";
    begin
        d_in.virt_mode <= '0';
        d_in.priv_mode <= '1';
        -- loadstore1 marks every aligned (non-quadword) access as both the
        -- first and last part of an atomic unit (loadstore1.vhdl atomic_first/last)
        d_in.atomic_first <= '1';
        d_in.atomic_last <= '1';
        m_in.valid <= '0';
        m_in.addr <= (others => '0');
        m_in.pte <= (others => '0');
        m_in.tlbie <= '0';
        m_in.doall <= '0';
        m_in.tlbld <= '0';
        rst <= '1';
        wait for 4*clk_period;
        rst <= '0';
        wait for 4*clk_period;
        wait until rising_edge(clk);

        -- 1. lwarx on a line the cache does NOT hold: one reserved memory read.
        report "EXT1: lwarx (miss) goes to memory marked ext_reserve";
        snapshot;
        do_access('1', '1', A, (others => '0'), x"FF");
        assert n_res_reads - res_reads0 = 1 and n_reads - reads0 = 1
            report "EXT1 FAIL: lwarx produced " & integer'image(n_reads - reads0) &
            " reads, " & integer'image(n_res_reads - res_reads0) & " with ext_reserve (want 1, 1)"
            severity failure;
        assert data = pattern(A / 8)
            report "EXT1 FAIL: lwarx data " & to_hstring(data) severity failure;

        -- 2. lwarx must not allocate: a plain load of the same line misses.
        report "EXT2: lwarx did not allocate the line";
        snapshot;
        do_access('1', '0', A, (others => '0'), x"FF");
        assert n_reads - reads0 >= 1 and n_res_reads - res_reads0 = 0
            report "EXT2 FAIL: load after lwarx did not go to memory (lwarx allocated the line)"
            severity failure;

        -- 3. lwarx on a line the cache DOES hold still asks memory.
        report "EXT3: lwarx (hit) still goes to memory marked ext_reserve";
        snapshot;
        do_access('1', '1', A, (others => '0'), x"FF");
        assert n_res_reads - res_reads0 = 1
            report "EXT3 FAIL: lwarx on a cached line was not sent to memory as reserved"
            severity failure;

        -- 4. stcx. that memory REJECTS: fails, memory and cached copy unchanged.
        report "EXT4: stcx. with ext_sc_fail = 1 fails";
        fail_next_sc <= '1';
        snapshot;
        do_access('0', '1', A, STDATA, x"0F");
        fail_next_sc <= '0';
        assert n_res_writes - res_writes0 = 1
            report "EXT4 FAIL: stcx. not presented to memory as a reserved write"
            severity failure;
        assert done = '0'
            report "EXT4 FAIL: stcx. reported SUCCESS although memory returned ext_sc_fail = 1"
            severity failure;
        snapshot;
        do_access('1', '0', A, (others => '0'), x"FF");     -- cached copy (line held)
        assert n_reads - reads0 = 0
            report "EXT4 FAIL: line not held - this check would not see the cached copy"
            severity failure;
        assert data = pattern(A / 8)
            report "EXT4 FAIL: failed stcx. modified the cached copy: " & to_hstring(data)
            severity failure;

        -- 5. stcx. that memory ACCEPTS: succeeds and updates the cached copy.
        report "EXT5: stcx. with ext_sc_fail = 0 succeeds";
        do_access('1', '1', A, (others => '0'), x"FF");     -- new reservation
        snapshot;
        do_access('0', '1', A, STDATA, x"0F");
        assert n_res_writes - res_writes0 = 1 and done = '1'
            report "EXT5 FAIL: accepted stcx. reported failure or was not a reserved write"
            severity failure;
        snapshot;
        do_access('1', '0', A, (others => '0'), x"FF");
        assert n_reads - reads0 = 0
            report "EXT5 FAIL: line not held - this check would not see the cached copy"
            severity failure;
        assert data = pattern(A / 8)(63 downto 32) & x"CAFEF00D"
            report "EXT5 FAIL: successful stcx. data not visible: " & to_hstring(data)
            severity failure;

        -- 6. stcx. with no reservation fails locally, nothing reaches memory.
        report "EXT6: stcx. without a reservation fails without a bus write";
        snapshot;
        do_access('0', '1', A, STDATA, x"0F");
        assert done = '0' and n_writes - writes0 = 0
            report "EXT6 FAIL: stcx. without reservation wrote to memory or succeeded"
            severity failure;

        -- 7. A snooped store does not kill the reservation: memory decides.
        report "EXT7: snooped store leaves the verdict to memory";
        do_access('1', '1', A, (others => '0'), x"FF");
        wait until rising_edge(clk);
        snoop.adr <= addr_to_wb(std_ulogic_vector(to_unsigned(A, 64)));
        snoop.cyc <= '1'; snoop.stb <= '1'; snoop.we <= '1';
        wait until rising_edge(clk);
        snoop.cyc <= '0'; snoop.stb <= '0'; snoop.we <= '0';
        for i in 1 to 4 loop
            wait until rising_edge(clk);
        end loop;
        snapshot;
        do_access('0', '1', A, STDATA, x"0F");
        assert n_res_writes - res_writes0 = 1 and done = '1'
            report "EXT7 FAIL: snooped store killed the reservation locally (stcx. never asked memory)"
            severity failure;

        -- 8. dcbz on a line the cache does NOT hold zeroes memory and does not
        --    allocate: the next load of that line must go to memory.
        report "EXT8: dcbz (miss) zeroes memory without allocating";
        snapshot;
        do_dcbz(16#200#);
        assert n_writes - writes0 = 2
            report "EXT8 FAIL: dcbz of a 16 B line wrote " & integer'image(n_writes - writes0) &
            " beats to memory (want 2)" severity failure;
        snapshot;
        do_access('1', '0', 16#208#, (others => '0'), x"FF");
        assert n_reads - reads0 >= 1
            report "EXT8 FAIL: load after a dcbz miss hit in the cache - dcbz allocated the line"
            severity failure;
        assert data = x"0000000000000000"
            report "EXT8 FAIL: memory not zeroed by dcbz: " & to_hstring(data) severity failure;

        -- 9. dcbz on a line the cache DOES hold zeroes the cached copy.
        report "EXT9: dcbz (hit) zeroes the held line";
        do_access('1', '0', 16#300#, (others => '0'), x"FF");   -- allocate by load
        snapshot;
        do_access('1', '0', 16#300#, (others => '0'), x"FF");
        assert n_reads - reads0 = 0
            report "EXT9 FAIL: setup - line 0x300 not held after a load" severity failure;
        do_dcbz(16#300#);
        snapshot;
        do_access('1', '0', 16#308#, (others => '0'), x"FF");
        assert n_reads - reads0 = 0 and data = x"0000000000000000"
            report "EXT9 FAIL: after dcbz hit, load missed or read " & to_hstring(data)
            severity failure;

        report "dcache_ext_atomics_tb: ALL CHECKS PASSED";
        std.env.finish;
    end process;
end;
