module tb ();
    //logic [riscv_pkg::XLEN-1:0] addr;
    //logic [riscv_pkg::XLEN-1:0] data;
    logic [riscv_pkg::XLEN-1:0] pc;
    logic                       update;
    logic                       clk;
    logic                       rstn;
    logic [riscv_pkg::XLEN-1:0] instr;
    logic [                4:0] reg_addr;
    logic [riscv_pkg::XLEN-1:0] reg_data;

    //test.log ile pc.log karşılaştırması yaparken daha rahat edebilmek için instr, reg_addr ve reg_data'yı ekledik ki program counter(pc) yanında bunları da
    //ekrana basalım ve test.log ile aynı formatta olsun.
    core_model i_core_model (
        .clk_i(clk),
        .rstn_i(rstn),
        .pc_o(pc),
        //.addr_i(addr),
        //.data_o(data),
        .update_o(update),
        .instr_o(instr),
        .reg_addr_o(reg_addr),
        .reg_data_o(reg_data)
    );

    initial begin
        #4;  //reset sinyalinin 0 olduğu süre kadar bekliyoruz çünkü reset 0 iken program sıfırlanıyor bu nedenle ekrana basmaya gerek yok.
        forever begin
            //Aşağıdaki if else bloğunda o anki program counter ı burada ekrana basıyoruz. 0x%8h sonunu 0 ile dolduracak şekilde 8 haneli hexadecimal olarak
            //ekrana basıyoruz. Bu sayede program counter ı takip edeceğiz. Eğer update değeri 1 ise yani bir güncelleme gerçekleştiyse ekrana basma işlemi
            //gerçekleştireceğiz bu şöyle olacak, eğer reg_addr 0 ise program counter(pc) ve instruction (instr) değerlerini ekrana basıyoruz. Eğer
            //reg_addr 0 değilse yani bir register güncellemesi varsa o zaman program counter (pc), instruction (instr), register adresi (reg_addr) ve
            //register data(reg_data) değerlerini ekrana basıyoruz.
            if(update) begin
                if(reg_addr == 0)
                    $display("0x%8h (0x%8h) ",pc, instr);
                else begin
                    if(reg_addr < 10)
                        $display("0x%8h (0x%8h) x%0d  0x%8h",pc, instr, reg_addr, reg_data); //aynı hizada olsun diye reg_addr sonrasında 2 boşluk bıraktık.
                    else
                        $display("0x%8h (0x%8h) x%0d 0x%8h",pc, instr, reg_addr, reg_data); // burada ise tek boşluk koyduk.
                end
                #2;  //clk her 2 birim saniyede değişiyor(0-1, 1-0) bu nedenle 2 birim de bir program counter ı ekrana basıyoruz.
            end
        end
    end

    initial begin  //burada clock sinyalini üretiyoruz
        clk = 0;
        forever #1 clk = ~clk;     //her 1 ns de clock sinyali değişiyor
    end

    initial begin
        rstn = 0;
        #4;
        rstn = 1; // 4 birim saniye sonra reset sinyali 1 oluyor ki program çalışmaya başlasın çünkü reset sinyali negatif edge ile çalışıyor
                  // bu nedenle reset sinyali 0 olduğunda program reset halinde oluyor.

        #4000; //10000 birim saniye bekliyoruz 

        //
        /*for(int i = 0; i<10; i++) begin
            addr = i;
            $display("data @ mem[0x%8h] = %8h", addr, data);  //memory'nin hangi adresinde hangi data varsa onu ekrana basıyoruz.
        end*/

        $finish;
    end

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0,tb);
    end

endmodule
