module core_model
import riscv_pkg::*;
(input  logic clk_i, rstn_i,          //instructor memory için CLK ve RST
 //input  logic [XLEN-1:0] addr_i,
 output logic            update_o,
 //output logic [XLEN-1:0] data_o,
 output logic [XLEN-1:0] pc_o,
 output logic [XLEN-1:0] instr_o,
 output logic [4:0]      reg_addr_o,
 output logic [XLEN-1:0] reg_data_o);


//XLEN ifadesi işlemcinin veriyolu genişliğini yani kaç bit olduğunu ifade ediyor. Burada XLEN 32 olduğu için 32 bitlik bir işlemciden bahsediyoruz.

//memory tanımlamaları
parameter int MEM_SIZE = 2048; // Memory tanımlamalarında bu parametreyi kullanacağız.
logic [31:0]     iMem [MEM_SIZE-1:0]; // instruction memory tanımlamasını yaptık
logic [31:0]     dMem [MEM_SIZE-1:0]; // data memory tanımlamasını yaptık
logic [XLEN-1:0] rf [31:0];       //32 bitlik 32 adet register file(rf) oluşturduk
initial $readmemh("./test/test.hex", iMem, 0, MEM_SIZE);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

assign pc_o       = pc_q_memory;
//assign data_o     = dMem[addr_i];
assign instr_o    = instr_q_memory;  //instr_d = iMem[pc_d[$clog2(MEM_SIZE) - 1 : 0]]; aşağıdaki programCounter_change_comb kısmında yandaki komut ile instr_d'yi okuduk zaten ve direkt instr_o'ya aktardık ki çıkışta görelim
assign reg_addr_o = rf_write_enable_d_writeback ? rd_d_writeback : '0;
assign reg_data_o = rd_data_d_writeback;
assign update_o   = rf_write_enable_d_writeback;


///////////////////////////////////////FETCH AŞAMASI//////////////////////////////////////////////////////////////////////////////////////

logic [XLEN-1:0] pc_d_fetch; // program counter'ın girişi. Yani sıradaki program counter değerini ifade eder.
logic [XLEN-1:0] pc_q_fetch; // program counter'ın çıkışı. Yani şu anki kullanılan program counter değerini ifade eder.
logic [XLEN-1:0] pc_q_fetch_to_decode;
logic            update_o_fetch;
logic [XLEN-1:0] instr_d_fetch;
logic [XLEN-1:0] instr_q_fetch;

always_ff @(posedge clk_i or negedge rstn_i) begin  : programCounter_change_flipFlop  // program counter'ın değiştiği blok
    if(!rstn_i) begin
        pc_q_fetch <= 'h8000_0000; // eğer reset sinyali 0 ise program counter'ına 0 değeri atanır yani program counter'ı resetlenir. PC'yi 8000_0000 yapmamızın sebebi
                             // SPIKE SIMULATORU'nun RISC-V'da program counter'ı 0x8000_0000 adresinden başlatmasıdır.
        update_o_fetch <= 0;       // eğer program counter'ın şimdiki değerine(pc_q) sonraki program counter(pc_d) ataması yapılmadıysa update_o = 0 yapılarak işlemin gerçekleşmediği gösterilir.
    end
    else begin
        pc_q_fetch <= pc_d_fetch;       // eğer reset inyali 1 ise program counter'ına sıradaki program counter'ı atanır,
        update_o_fetch <= 1;     //eğer program counter'ın şimdiki değerine(pc_q) sonraki program counter(pc_d) ataması yapıldıysa update_o = 1 yapılarak bu işlemin gerçekleşmiş olduğu gösterilir.
    end
end

always_ff @(posedge clk_i or negedge rstn_i) begin : programCounter_fetch_to_register_flipFLop
    if(!rstn_i) begin
        pc_q_fetch_to_decode <= '0; // eğer reset sinyali 0 ise program counter'ını sıfırla yani program counter'ı resetle.
    end
    else begin
        pc_q_fetch_to_decode <= pc_q_fetch; // eğer reset sinyali 1 ise sıradaki program counter'ı pc_q'den al ve pc_q_fetch_to_decode'ya ata.
    end
end


always_ff @(posedge clk_i or negedge rstn_i) begin : instr_d_flipFlop
    if(!rstn_i) begin
        instr_q_fetch <= '0; // eğer reset sinyali 0 ise instruction'ı sıfırla yani instruction'ı resetle.
    end
    else begin
        instr_q_fetch <= instr_d_fetch; // eğer reset sinyali 1 ise sıradaki instruction'ı pc_q'den al ve instr_d'ye ata.
    end
end

always_comb begin : programCounter_change_comb
    //pc_d_fetch = pc_q_fetch;
    if(jump_pc_valid_d_execute)        // eğer jump_pc_valid_d değeri 1 olduysa dışarıdan komut gelmiştir ve başka bir program counter(pc) gelmiştir o halde işlemci counter'ını normal
        pc_d_fetch = jump_pc_d_execute;      // şekilde işletmek yerine dışarıdan gelen bu program counter'ı pc_d girişine atıyoruz ve bu sayede yeni program counter'ı çıkışa aktarılıyor 
    else                       // yukarıdaki pc_q <= pc_d sayesinde
        pc_d_fetch = pc_q_fetch + 4;       //burada ise yeni program counter eski program counter'ın üstüne + 4 eklenmiş hali olarak devam ediyor.
    
    instr_d_fetch = iMem[pc_q_fetch[$clog2(MEM_SIZE*4) - 1 : 2]]; /* Burada pc_q[$clog2(MEM_SIZE-1):2] ifadesi şunun için var, pc_q yukarıda [XLEN-1:0] şeklinde tanımlandı yani 32 bit, fakat bizim Memory Size'ımız 1024 yani
    adresleme yapmak için 32 bite değil yalnızca 10 bite ihtiyacımız var bu nedenle $clog2 fonksiyonunu kullanarak pc_q'yi 32 bitten 10 bite indirgiyoruz. Bu nasıl oluyor $clog2(parametre) fonksiyonu
    bir sayının 2 tabanındaki logaritmasını hesaplar, bu nedenle $clog2(MEM_SIZE-1) ifadesi MEM_SIZE değeri 1024 olduğu için 1024-1 = 1023 sayısının 2 tabanındaki logaritmasını hesaplar bu da 10'a eşittir.
    Bu durumda pc_q[$clog2(MEM_SIZE-1) - 1 : 0] ifadesi aslında pc_d[(10-1):0] => pc_q[9:0] şekline indirgenmiş oluyor yani pc_q(sıradaki program counter) değerinin SON 10 BİTİNE indirgenmiş olduk.*/

    /*NOT: instr_d = iMem[pc_q[$clog2(MEM_SIZE) - 1 : 0]]; değil de instr_d = iMem[pc_q[$clog2(MEM_SIZE) - 1 : 2]]; yapmamızın sebebi iMem'in logic [31:0]     iMem [MEM_SIZE-1:0]; şeklinde WORD ADRESLEME
    yapmasından kaynaklı yani iMem[0] = {byte3, byte2, byte1, byte0}, iMem[1] = {byte7,byte6,byte5,byte4} gibi. instr_d = iMem[pc_q[$clog2(MEM_SIZE) - 1 : 2]]; risc-v'da bu şekilde alt iki biti çıkardığımız zaman 
    BYTE ADRESLEME MODUNDAN WORD ADRESLEME MODUNA GEÇİYORUZ. Eğer bunu yapmazsak mesela adres 00000004 olduğunda iMem[1]'e erişmesi gerekirken iMem[4]'e erişir bu da yanlıştır çünkü 00000004 ifadesi 4. byte'ı
    yani iMem[1]'in başlangıç adresini ifade eder.*/

    /* MEM_SIZE'ı 4 ile çarpmamızın sebebi, MEM_SIZE bize aslında KAÇ ADET WORD olduğunu gösteriyor yani 1024 adet WORD'umuz var fakat bir WORD İÇİNDE 4 BYTE OLDUĞU İÇİN  MEM_SIZE * 4 yapıyoruz ve belleğin
    toplam boyutunu BYTE CİNSİNDEN HESAPLIYORUZ Kİ PROGRAM COUNTER BYTE BYTE İLERLEDİĞİ İÇİN TOPLAM BYTE ALANINI HESAPLAMIŞ OLALIM.*/
end
///////////////////////////////////////FETCH AŞAMASI//////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////DECODE AŞAMASI//////////////////////////////////////////////////////////////////////////////////////

    logic [XLEN-1:0] imm_data_d_decode;
    logic [XLEN-1:0] imm_data_q_decode;
    logic [XLEN-1:0] rs1_data_d_decode;
    logic [XLEN-1:0] rs1_data_q_decode;
    logic [XLEN-1:0] rs2_data_d_decode;
    logic [XLEN-1:0] rs2_data_q_decode;
    logic [     4:0] shamt_data_d_decode;
    logic [     4:0] shamt_data_q_decode;
    logic [XLEN-1:0] instr_d_decode;
    assign instr_d_decode = instr_q_fetch; //instr_d_decode, fetch aşamasında elde edilen instr_q_fetch değerini alır.
    logic [XLEN-1:0] instr_q_decode;
    logic [XLEN-1:0] pc_d_decode;
    assign pc_d_decode = pc_q_fetch_to_decode; //pc_d_decode, fetch aşamasında elde edilen pc_q_fetch değerini alır.
    logic [XLEN-1:0] pc_q_decode;

    logic [     4:0] rs1_addr_d_decode;
    assign rs1_addr_d_decode = instr_d_decode[19:15];
    logic [     4:0] rs1_addr_q_decode;
    
    logic [     4:0] rs2_addr_d_decode;
    assign rs2_addr_d_decode = instr_d_decode[24:20];
    logic [     4:0] rs2_addr_q_decode;


always_comb begin : decode_block   //instr_d içindeki instruction'un decode edilmesi, bu aşamada instr_d'nin belirli kısımlarındaki OPCODE'ların ne olduğuna göre case'ler açıp o case'ler içinde o OPCODE'nin görevi
                                   //neyse onları yapıcaz. Opcode dediğimiz kısım istr_d'nin son 7 biti yani instr_d[6:0] kısmı.
    
    case(instr_d_decode[6:0])
        OpcodeLui:                 //Lui = Load Upper Immediate
            imm_data_d_decode = {instr_d_decode[31:12], 12'b0};  //LUI komutunun tanımına göre imm_data_d'ya atama yaptık. Tanımda instruction'un 31:12 bitini al ve geri kalanına 0 ata diyor o nedenle biz de öyle yaptık.
        OpcodeAuipc:
            imm_data_d_decode = {instr_d_decode[31:12], 12'b0};   
        OpcodeJal: begin
            imm_data_d_decode[31:21] = {11{instr_d_decode[31]}};
            imm_data_d_decode[20]    = instr_d_decode[31];
            imm_data_d_decode[10:1]  = instr_d_decode[30:21];
            imm_data_d_decode[11]    = instr_d_decode[20];
            imm_data_d_decode[19:12] = instr_d_decode[19:12];
            imm_data_d_decode[0]     = 1'b0; 
        end
        OpcodeJalr:
            if(instr_d_decode[14:12] == F3_JALR) begin
                rs1_data_d_decode = rf[instr_d_decode[19:15]];
                imm_data_d_decode[31:12] = {20{instr_d_decode[31]}};
                imm_data_d_decode[11:0]  = instr_d_decode[31:20];
            end
        

        OpcodeBranch:                  //OpcodeBranch birçok operasyonda aynı olduğu için bir alt case içinde instr_d'nin farklı bir kısmına bakıp branch operasyonlarını ona göre ayırıyoruz.
            case(instr_d_decode[14:12])       //FUNCT3 koduna bakıyoruz
                F3_BEQ, F3_BNE, F3_BLT, F3_BGE, F3_BLTU, F3_BGEU: begin // buradaki bütün case'ler için imm_data_d, rs1, rs2 ataması aynı burada fark YALNIZCA FUNCT3(instr_d[14:12]) kısmında olduğu için
                                                                //ayrım burada olucak BEQ,BNE, BLT, .... hangisiyse artık gerisi hepsi için aynı olduğu için böyle tek satırda yazdık.
                    rs1_data_d_decode        = rf[instr_d_decode[19:15]];
                    rs2_data_d_decode        = rf[instr_d_decode[24:20]];
                    imm_data_d_decode[31:13] = {19{instr_d_decode[31]}};   // riscv instruction pdf'indeki talimatlara göre IMM DATA'NIN HANGİ BİTİNİN HANGİ INSTRUCTION BITLERİNE KARŞILIK GELDİĞİNİ belirtiyoruz.
                    imm_data_d_decode[12]    = instr_d_decode[31];
                    imm_data_d_decode[11]    = instr_d_decode[7];
                    imm_data_d_decode[10:5]  = instr_d_decode[30:25];
                    imm_data_d_decode[4:1]   = instr_d_decode[11:8];
                    imm_data_d_decode[0]     = 1'b0;
                end
                default: ;
            endcase
        OpcodeLoad:                  //aynı şekilde opcodebranch'daki olayın aynısını burada yapıyoruz.
            case(instr_d_decode[14:12])     //FUNCT3 koduna bakıyoruz
                F3_LB, F3_LH, F3_LW, F3_LBU, F3_LHU  : begin
                    rs1_data_d_decode        = rf[instr_d_decode[19:15]];
                    imm_data_d_decode[31:12] = {20{instr_d_decode[31]}};
                    imm_data_d_decode[11:0]  = instr_d_decode[31:20];
                end
                default: ;
            endcase
        OpcodeStore:
            case(instr_d_decode[14:12])     //FUNCT3 koduna bakıyoruz
                F3_SB, F3_SH, F3_SW   : begin
                    rs1_data_d_decode        = rf[instr_d_decode[19:15]];
                    rs2_data_d_decode        = rf[instr_d_decode[24:20]];
                    imm_data_d_decode[31:12] = {20{instr_d_decode[31]}};
                    imm_data_d_decode[11:5]  = instr_d_decode[31:25];
                    imm_data_d_decode[4:0]   = instr_d_decode[11:7];
                end
                default: ;
            endcase
        OpcodeOpImm:
            case(instr_d_decode[14:12])     //FUNCT3 koduna bakıyoruz
            //ADD IMMEDIATE, sonunda I varsa Immediate, yoksa mesela F3_ADD ise normal ADD işlemi. Diğer kodlar için de böyle.
                F3_ADDI, F3_SLTI, F3_SLTIU, F3_XORI, F3_ORI, F3_ANDI : begin
                    rs1_data_d_decode        = rf[instr_d_decode[19:15]];
                    imm_data_d_decode[31:12] = {20{instr_d_decode[31]}};
                    imm_data_d_decode[11:0]  = instr_d_decode[31:20]; 
                end
                F3_SLLI :
                    if(instr_d_decode[31:25] == F7_SLLI) begin
                        shamt_data_d_decode = instr_d_decode[24:20];
                        rs1_data_d_decode   = rf[instr_d_decode[19:15]];          
                    end
                F3_SRLI_SRAI :
                    if(instr_d_decode[31:25] == F7_SRLI) begin
                        shamt_data_d_decode = instr_d_decode[24:20];
                        rs1_data_d_decode   = rf[instr_d_decode[19:15]];
                    end
                    else if(instr_d_decode[31:25] == F7_SRAI) begin
                        shamt_data_d_decode = instr_d_decode[24:20];
                        rs1_data_d_decode   = rf[instr_d_decode[19:15]];
                    end
                default: ;
            endcase
        OpcodeOp:
            case(instr_d_decode[14:12])
                F3_ADD_SUB :
                    if(instr_d_decode[31:25] == F7_ADD) begin
                        rs1_data_d_decode   = rf[instr_d_decode[19:15]];
                        rs2_data_d_decode   = rf[instr_d_decode[24:20]];
                    end else if(instr_d_decode[31:25] == F7_SUB) begin
                        rs1_data_d_decode   = rf[instr_d_decode[19:15]];
                        rs2_data_d_decode   = rf[instr_d_decode[24:20]];
                    end
                F3_SLL, F3_SLT, F3_SLTU, F3_XOR, F3_OR, F3_AND : begin
                    rs1_data_d_decode   = rf[instr_d_decode[19:15]];
                    rs2_data_d_decode   = rf[instr_d_decode[24:20]];
                end
                F3_SRL_SRA :
                    if(instr_d_decode[31:25] == F7_SRL) begin
                        rs1_data_d_decode   = rf[instr_d_decode[19:15]];
                        rs2_data_d_decode   = rf[instr_d_decode[24:20]];                            
                    end else if(instr_d_decode[31:25] == F7_SRA) begin
                        rs1_data_d_decode   = rf[instr_d_decode[19:15]];
                        rs2_data_d_decode   = rf[instr_d_decode[24:20]];
                    end
                default: ;
            endcase
        default: ;
    endcase
end

always_ff @(posedge clk_i or negedge rstn_i) begin : decode_to_execute_ff
    if(!rstn_i) begin
        imm_data_q_decode <= '0;
        rs1_data_q_decode <= '0;
        rs2_data_q_decode <= '0;
        shamt_data_q_decode <= '0;
        instr_q_decode <= '0;
        pc_q_decode <= '0;
        rs1_addr_q_decode <= '0;
        rs2_addr_q_decode <= '0;
    end else begin
        imm_data_q_decode <= imm_data_d_decode;
        rs1_data_q_decode <= rs1_data_d_decode;
        rs2_data_q_decode <= rs2_data_d_decode;
        shamt_data_q_decode <= shamt_data_d_decode;
        instr_q_decode <= instr_d_decode;
        pc_q_decode <= pc_d_decode;
        rs1_addr_q_decode <= rs1_addr_d_decode;
        rs2_addr_q_decode <= rs2_addr_d_decode; 
    end
end
//////////////////////////////////////////DECODE AŞAMASI//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////EXECUTE AŞAMASI//////////////////////////////////////////////////////////////////////////////////////

    logic [XLEN-1:0] pc_d_execute;
    assign pc_d_execute = pc_q_decode; //pc_d_execute, decode aşamasında elde edilen pc_q_decode değerini alır.
    logic [XLEN-1:0] pc_q_execute;
    logic jump_pc_valid_d_execute;
    logic [XLEN-1:0] jump_pc_d_execute;
    logic [XLEN-1:0] imm_data_d_execute;
    assign imm_data_d_execute = imm_data_q_decode; //imm_data_d_execute, decode aşamasında elde edilen imm_data_q_decode değerini alır.
    logic [XLEN-1:0] instr_d_execute;
    assign instr_d_execute = instr_q_decode; //instr_d_execute, decode aşamasında elde edilen instr_q_decode değerini alır.
    logic [XLEN-1:0] instr_q_execute;
    logic [XLEN-1:0] rs1_data_d_execute;
    logic [XLEN-1:0] rs2_data_d_execute;
    logic [4:0] shamt_data_d_execute;
    assign shamt_data_d_execute = shamt_data_q_decode; //shamt_data_d_execute, decode aşamasında elde edilen shamt_data_q_decode değerini alır.
    logic [XLEN-1:0] rd_data_d_execute; // execute aşamasındaki register destination data
    logic [XLEN-1:0] rd_data_q_execute; // execute aşamasındaki register destination data'nın çıkışı
    logic [     4:0] rd_d_execute;      // execute aşamasındaki register destination yani yazılacak register adresi
    assign rd_d_execute = instr_d_execute[11:7]; // rd_d_execute, decode aşamasında elde edilen instr_d_execute değerinin 11:7 bitlerini alır.
    logic [     4:0] rd_q_execute;      // execute aşamasındaki register destination yani yazılacak register adresinin çıkışı 
    logic rf_write_enable_d_execute;    // Register file'a yazma işlemini kontrol eden sinyal.
    logic rf_write_enable_q_execute;    // rf_write_enable değerinin çıkışı.
    
    
    logic [XLEN-1:0] memory_write_data_d_execute;     // Store aşamasında hafızaya yazacağımız veriyi ifade eder.
    logic [XLEN-1:0] memory_write_data_q_execute;     // Hafızaya yazacağımız verinin çıkışı.
    logic [XLEN-1:0] memory_write_addr_d_execute; //Belleğin hangi adresine yazacaksak o adresi hesaplamak için gerekli değişken.
    logic [XLEN-1:0] memory_write_addr_q_execute; // Hafızaya yazacağımız adresin çıkışı.
    logic            memory_write_enable_d_execute; // Hafızaya yazabilmemiz için hafızanın write enable'ının 1 olması gerekir.
    logic            memory_write_enable_q_execute; // Hafızaya yazma işleminin gerçekleşip gerçekleşmediğini kontrol eden sinyal.

    always_comb begin : forward_comb
        
        //rs1 forward
        if(forward_rs1 == 2'b01)
            rs1_data_d_execute = rd_data_q_execute;
        else if (forward_rs1 == 2'b10)
            rs1_data_d_execute = rd_data_q_memory;
        else
            rs1_data_d_execute = rs1_data_q_decode;

        //rs2 forward
        if(forward_rs2 == 2'b01)
            rs2_data_d_execute = rd_data_q_execute;  // execute aşamasından forward
        else if(forward_rs2 == 2'b10)
            rs2_data_d_execute = rd_data_q_memory;   // memory aşamasından forward
        else
            rs2_data_d_execute = rs2_data_q_decode;  // forward yok
    end

always_comb begin : execute_block  //
    // Burada eğer herhangi bir case durumuna girmezsek, bir önceki değeri tutup latch oluşturmasın diye en başta bütün verileri sıfırlıyoruz.
    jump_pc_valid_d_execute     = 0;
    jump_pc_d_execute           = 0;
    rd_data_d_execute             = 0;
    rf_write_enable_d_execute     = 0;
    memory_write_enable_d_execute = 0;
    memory_write_data_d_execute   = 0;
    memory_write_addr_d_execute   = 0;
    //----------------------------------------------------------------------------------------------------------------------------------------- 
    case(instr_d_execute[6:0])
        OpcodeLui: begin
            rd_data_d_execute = imm_data_d_execute;          //register destination data, yani hedef register verisi imm_data_d olarak ayarlandı.
            rf_write_enable_d_execute = 1'b1;      //rf_write_enable_d_execute 1 olarak ayarlandı ki yazma işlemi gerçekleşebilsin.
        end
        OpcodeAuipc: begin
            rd_data_d_execute = pc_d_execute + imm_data_d_execute;   //register destination data, yani hedef register verisi imm_data_d + bir sonraki program counter olarak ayarlandı.
            rf_write_enable_d_execute = 1'b1;
        end
        OpcodeJal: begin
            jump_pc_valid_d_execute = 1'b1;
            jump_pc_d_execute = imm_data_d_execute + pc_d_execute;
            rd_data_d_execute = pc_d_execute + 4;
            rf_write_enable_d_execute = 1'b1;      //rf_write_enable_d_execute 1 olarak ayarlandı ki yazma işlemi gerçekleşebilsin.
        end
        OpcodeJalr: begin
            jump_pc_valid_d_execute = 1'b1;
            jump_pc_d_execute = imm_data_d_execute + rs1_data_d_execute;
            jump_pc_d_execute[0] = 1'b0; // Burada en son bit 0 yapılıyor çünkü JALR komutunda en son bitin 0 olması gerekiyor.
            rd_data_d_execute = pc_d_execute + 4;
            rf_write_enable_d_execute = 1'b1;      //rf_write_enable_d_execute 1 olarak ayarlandı ki yazma işlemi gerçekleşebilsin.
        end
        OpcodeBranch:
            case(instr_d_execute[14:12])
             F3_BEQ: if(rs1_data_d_execute == rs2_data_d_execute) begin   // BRANCH IF EQUAL
                jump_pc_d_execute = pc_d_execute + imm_data_d_execute;
                jump_pc_valid_d_execute = 1'b1;
             end
             F3_BNE: if(rs1_data_d_execute != rs2_data_d_execute) begin   // BRANCH IF NOT EQUAL
                jump_pc_d_execute = pc_d_execute + imm_data_d_execute;
                jump_pc_valid_d_execute = 1'b1; 
             end 
             F3_BLT: if($signed(rs1_data_d_execute) < $signed(rs2_data_d_execute)) begin   // BRANCH IF LESS THAN, systemverilog'da signed ifadelerin gösterimi $signed şeklinde bu nedenle böyle ifade ettik.
                jump_pc_d_execute = pc_d_execute + imm_data_d_execute;
                jump_pc_valid_d_execute = 1'b1; 
             end 
             F3_BGE: if($signed(rs1_data_d_execute) >= $signed(rs2_data_d_execute)) begin   // BRANCH IF GREATER OR EQUAL, systemverilog'da signed ifadelerin gösterimi $signed şeklinde bu nedenle böyle ifade ettik.
                jump_pc_d_execute = pc_d_execute + imm_data_d_execute;
                jump_pc_valid_d_execute = 1'b1; 
             end 
             F3_BLTU: if(rs1_data_d_execute < rs2_data_d_execute) begin   // BRANCH IF LESS THAN(UNSIGNED)
                jump_pc_d_execute = pc_d_execute + imm_data_d_execute;
                jump_pc_valid_d_execute = 1'b1; 
             end 
             F3_BGEU: if(rs1_data_d_execute >= rs2_data_d_execute) begin   // BRANCH IF GREATER OR EQUAL(UNSIGNED)
                jump_pc_d_execute = pc_d_execute + imm_data_d_execute;
                jump_pc_valid_d_execute = 1'b1; 
             end
             default: ; 
            endcase
        OpcodeLoad:
            case(instr_d_execute[14:12])
             F3_LB: begin   //LOAD BYTE
                rd_data_d_execute[31:8]   = {24{dMem[rs1_data_d_execute[$clog2(MEM_SIZE)-1:0]][7]}}; //burada ise sign extension yapıyoruz çünkü 32 bitlik rd_data var ve biz alt satırda bu 32 bitlik data içine sadece
                                                                                 //dMem içerisindeki eriştiğimiz adres içindeki verinin son 8 bitini aktardık çünkü Load işlemleri sadece son 8 biti ister.

                rd_data_d_execute[7:0]    = dMem[rs1_data_d_execute[$clog2(MEM_SIZE)-1:0]][7:0];     //rs1_data[$clog2[MEM_SIZE]-1:0] bu ifade ile rs1_data[9:0] şeklinde 10 bitlik bir değer gelicek ve bunun sayesinde dMem
                                                                                 //içerisindeki 0-1023 adresten birine erişicez sonrasında bu adres içindeki [7:0] 8 bitlik veriyi rd_data[7:0] a aktarıcaz çünkü
                                                                                 //LB(LOAD BYTE) yapıyoruz yani 1 byte(8 bit) lik bir yükleme yapıyoruz bu nedenle son 8 biti[7:0] çekiyoruz.
                rf_write_enable_d_execute = 1'b1;
             end
             F3_LH: begin  //LOAD HALFBYTE
                rd_data_d_execute[31:16]  = {16{dMem[rs1_data_d_execute[$clog2(MEM_SIZE)-1:0]][15]}};//yine aynı şekilde sign extension yapıyoruz.
                rd_data_d_execute[15:0]   = dMem[rs1_data_d_execute[$clog2(MEM_SIZE)-1:0]][15:0];    //burada da yukarıdaki LB komutundaki açıklamalar aynen geçerli, yalnız burada LH(LOAD HALFWORD) yapıyoruz yani 2 byte(16 bit)
                                                                                 //yükleme yapıyoruz bu nedenle son 16 biti[15:0] çekiyoruz.
                rf_write_enable_d_execute = 1'b1;
             end
             F3_LW: begin  //LOAD WORD
                rd_data_d_execute         = dMem[rs1_data_d_execute[$clog2(MEM_SIZE)-1:0]];          //burada da yukarıdaki LB komutundaki açıklamalar aynen geçerli, yalnız burada LW(LOAD WORD) yapıyoruz yani 4 byte(32 bit)
                                                                                 //yükleme yapıyoruz, zaten 32 bit yüklediğimiz için herhangi bir EXTENSION YAPMAMIZA GEREK KALMIYOR.
                rf_write_enable_d_execute = 1'b1;
             end
             F3_LBU: begin //LOAD BYTE UNSIGNED
                rd_data_d_execute[31:8] = 24'b0;                                           //burada da yukarıdaki LB komutundaki açıklamalar aynen geçerli lakin burada LBU(UNSIGNED) olduğu için extension'u
                                                                                 //sign extension yapmıyoruz de ZERO(0) EXTENSION olarak yapıyoruz.
                rd_data_d_execute[7:0]  = dMem[rs1_data_d_execute[$clog2(MEM_SIZE)-1:0]][7:0];
                rf_write_enable_d_execute = 1'b1;
             end
             F3_LHU: begin //LOAD HALFWORD UNSIGNED
                rd_data_d_execute[31:16]  = 16'b0;                                         //Yine SIGN EXTENSION YERINE ZERO EXTENSION YAPTIK, çünkü komut LHU(UNSIGNED) olarak ifade edilmiş gerisi LH ile aynı.
                rd_data_d_execute[15:0]   = dMem[rs1_data_d_execute[$clog2(MEM_SIZE)-1:0]][15:0];    //burada da yukarıdaki LB komutundaki açıklamalar aynen geçerli, yalnız burada LH(LOAD HALFWORD) yapıyoruz yani 2 byte(16 bit)
                                                                                 //yükleme yapıyoruz bu nedenle son 16 biti[15:0] çekiyoruz.
                rf_write_enable_d_execute = 1'b1;
             end
             default: ;
            endcase
        OpcodeStore:
            case(instr_d_execute[14:12])
             F3_SB: begin
                memory_write_enable_d_execute  = 1'b1;
                memory_write_data_d_execute    = rs2_data_d_execute;            //yine risc-v instruction setindeki talimatlara göre memory_write_data_d_execute = rs2_data atamasını yaptık.
                memory_write_addr_d_execute    = rs1_data_d_execute + imm_data_d_execute; //rs1_data => rs1 register'ının adresi(base adres), imm_data_d => instruction'dan gelen offset(kayma) değeri, bu ikisi toplanınca memory_write_adress
                                                            //bulunmuş oluyor, bu adrese ise memory_write_data_d_execute yani rs2_data değeri yazılıyor.
             end
             F3_SH: begin
                memory_write_enable_d_execute  = 1'b1;
                memory_write_data_d_execute    = rs2_data_d_execute;  //yine risc-v instruction setindeki talimatlara göre memory_write_data_d_execute = rs2_data atamasını yaptık.
                memory_write_addr_d_execute    = rs1_data_d_execute + imm_data_d_execute;
             end
             F3_SW: begin
                memory_write_enable_d_execute  = 1'b1;
                memory_write_data_d_execute    = rs2_data_d_execute;  //yine risc-v instruction setindeki talimatlara göre memory_write_data_d_execute = rs2_data atamasını yaptık.
                memory_write_addr_d_execute    = rs1_data_d_execute + imm_data_d_execute;
             end
             default: ;
            endcase
        OpcodeOpImm:
            case(instr_d_execute[14:12])
             F3_ADDI: begin
                rf_write_enable_d_execute = 1'b1;
                rd_data_d_execute = $signed(imm_data_d_execute) + $signed(rs1_data_d_execute);
             end
             F3_SLTI: begin  //SET LESS THAN IMMEDIATE
                rf_write_enable_d_execute = 1'b1;
                if($signed(rs1_data_d_execute) < $signed(imm_data_d_execute))
                    rd_data_d_execute = 32'b1;
             end
             F3_SLTIU: begin //SET LESS THAN IMMEDIATE(UNSIGNED)
                rf_write_enable_d_execute = 1'b1;
                if(rs1_data_d_execute < imm_data_d_execute)
                    rd_data_d_execute = 32'b1;
             end
             F3_XORI: begin
                rf_write_enable_d_execute = 1'b1;
                rd_data_d_execute = rs1_data_d_execute ^ imm_data_d_execute;
             end
             F3_ORI: begin
                rf_write_enable_d_execute = 1'b1;
                rd_data_d_execute = rs1_data_d_execute | imm_data_d_execute;
             end
             F3_ANDI: begin
                rf_write_enable_d_execute = 1'b1;
                rd_data_d_execute = rs1_data_d_execute & imm_data_d_execute;
             end
             F3_SLLI:     //SHIFT LEFT LOGICAL IMMEDIATE
                if(instr_d_execute[31:25] == F7_SLLI) begin
                    rf_write_enable_d_execute = 1'b1;
                    rd_data_d_execute = rs1_data_d_execute << shamt_data_d_execute;
                end
            F3_SRLI_SRAI:     
                if(instr_d_execute[31:25] == F7_SRLI) begin          //SHIFT RIGHT LOGICAL IMMEDIATE
                    rf_write_enable_d_execute = 1'b1;
                    rd_data_d_execute = rs1_data_d_execute >> shamt_data_d_execute;
                end else if(instr_d_execute[31:25] == F7_SRAI) begin //SHIFT RIGHT ARITMATIC IMMEDIATE
                    rf_write_enable_d_execute = 1'b1;
                    rd_data_d_execute = rs1_data_d_execute >>> shamt_data_d_execute;
                end
            default: ;
            endcase
        OpcodeOp:
            case(instr_d_execute[14:12])
             F3_ADD_SUB:
                if(instr_d_execute[31:25] == F7_ADD) begin
                    rf_write_enable_d_execute = 1'b1;
                    rd_data_d_execute = rs1_data_d_execute + rs2_data_d_execute;
                end else if(instr_d_execute[31:25] == F7_SUB) begin
                    rf_write_enable_d_execute = 1'b1;
                    rd_data_d_execute = rs1_data_d_execute - rs2_data_d_execute;
                end
            F3_SLL:     begin   //SHIFT LEFT LOGICAL
                rf_write_enable_d_execute = 1'b1;
                rd_data_d_execute = rs1_data_d_execute << rs2_data_d_execute;
            end
            F3_SLT:     begin   //SET LESS THAN
                rf_write_enable_d_execute = 1'b1;
                if($signed(rs1_data_d_execute) < $signed(rs2_data_d_execute))
                    rd_data_d_execute = 32'b1;
            end
            F3_SLTU:    begin  //SET LESS THAN UNSIGNED
                rf_write_enable_d_execute = 1'b1;
                if(rs1_data_d_execute < rs2_data_d_execute)
                    rd_data_d_execute = 32'b1;
            end
            F3_XOR:     begin
                rf_write_enable_d_execute = 1'b1;
                rd_data_d_execute = rs1_data_d_execute ^ rs2_data_d_execute;
            end
            F3_SRL_SRA: 
                if(instr_d_execute[31:25] == F7_SRL) begin //SHIFT RIGHT LOGICAL
                    rf_write_enable_d_execute = 1'b1;
                    rd_data_d_execute = rs1_data_d_execute >> rs2_data_d_execute;
                end else if(instr_d_execute[31:25] == F7_SRA) begin
                    rf_write_enable_d_execute = 1'b1;
                    rd_data_d_execute = $signed(rs1_data_d_execute) >>> rs2_data_d_execute; //Aritmetik şekilde sağa kaydırmak için hem >>> hem de $signed ifadesi kullanmamız lazım aynı anda
                end
            F3_OR: begin
                rf_write_enable_d_execute = 1'b1;
                rd_data_d_execute = rs1_data_d_execute | rs2_data_d_execute;                  
            end
            F3_AND: begin
                rf_write_enable_d_execute = 1'b1;
                rd_data_d_execute = rs1_data_d_execute & rs2_data_d_execute;
            end
            default: ;
        endcase
        default: ;
    endcase


end

always_ff @(posedge clk_i or negedge rstn_i) begin : execute_to_memory_ff
    if(!rstn_i) begin
        pc_q_execute <= '0;
        instr_q_execute <= '0;
        rd_data_q_execute <= '0;
        rd_q_execute <= '0;
        rf_write_enable_q_execute <= '0;
        memory_write_data_q_execute <= '0;
        memory_write_addr_q_execute <= '0;
        memory_write_enable_q_execute <= '0;
    end else begin
        pc_q_execute <= pc_d_execute;
        instr_q_execute <= instr_d_execute;
        rd_data_q_execute <= rd_data_d_execute;
        rd_q_execute <= rd_d_execute;
        rf_write_enable_q_execute <= rf_write_enable_d_execute;
        memory_write_data_q_execute <= memory_write_data_d_execute;
        memory_write_addr_q_execute <= memory_write_addr_d_execute;
        memory_write_enable_q_execute <= memory_write_enable_d_execute;
    end
end
//////////////////////////////////////////EXECUTE AŞAMASI//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////MEMORY AŞAMASI//////////////////////////////////////////////////////////////////////////////////////////////

    logic [XLEN-1:0] pc_d_memory;
    assign pc_d_memory = pc_q_execute; //pc_d_memory, execute aşamasında elde edilen pc_q_execute değerini alır.
    logic [XLEN-1:0] pc_q_memory;
    logic [XLEN-1:0] instr_d_memory;
    assign instr_d_memory = instr_q_execute; //instr_d_memory, execute aşamasında elde edilen instr_q_execute değerini alır.
    logic [XLEN-1:0] instr_q_memory;
    
    logic            memory_write_enable_d_memory;
    assign memory_write_enable_d_memory = memory_write_enable_q_execute;
    logic [XLEN-1:0] memory_write_addr_d_memory;
    assign memory_write_addr_d_memory = memory_write_addr_q_execute;
    logic [XLEN-1:0] memory_write_data_d_memory;
    assign memory_write_data_d_memory = memory_write_data_q_execute;

    logic [XLEN-1:0] rd_data_d_memory;
    assign rd_data_d_memory = rd_data_q_execute;
    logic [XLEN-1:0] rd_data_q_memory;
    logic [     4:0] rd_d_memory;
    assign rd_d_memory = rd_q_execute; //rd_d_memory, execute aşamasında elde edilen rd_q_execute değerini alır.
    logic [     4:0] rd_q_memory;

    logic rf_write_enable_d_memory;
    assign rf_write_enable_d_memory = rf_write_enable_q_execute; //rf_write_enable_d_memory, execute aşamasında elde edilen rf_write_enable_q_execute değerini alır.
    logic rf_write_enable_q_memory;

always_ff @(posedge clk_i or negedge rstn_i) begin   //Store işlemi için gerekli kod bloğu
    if(!rstn_i) begin
    end
    else if(memory_write_enable_d_memory) begin
        //sanki instr_d[14:12] değeri aşağıdaki işlemlerden birine denk gelirse store işlemi olsa da olmasa da aşağıdaki işlemlerden biri gerçekleşiyor gibi gözüküyor fakat bu durumu memory_write_enable_d_execute
        //engeliyor çünkü eğer store varsa decode aşamasında memory_write_enable_d_execute değeri 1 olacak, yoksa 0 olarak kalacağı için aşağıdaki case'lerden herhangi birine girmeyecek. 
        //memory_write_addr_d_execute değeri ise execute aşamasındaki store komutlarının altındaki kod bloklarında belirleniyor.
        case(instr_d_memory[14:12]) 
         F3_SB: dMem[memory_write_addr_d_memory[$clog2(MEM_SIZE)-1:0]][7:0]  <= memory_write_data_d_memory[7:0];  //yine $clog2(MEM_SIZE) kullanarak adres değerini [9:0] a düşürdük yani 10 bitlik(maks 1024) olabilecek şekilde ayarladık
         F3_SH: dMem[memory_write_addr_d_memory[$clog2(MEM_SIZE)-1:0]][15:0] <= memory_write_data_d_memory[15:0]; //çünkü MEM_SIZE değerimiz 1024 yani maksimum 1024 adet adresimiz var [0-1023] arası.
         F3_SW: dMem[memory_write_addr_d_memory[$clog2(MEM_SIZE)-1:0]]       <= memory_write_data_d_memory;
         default: ;
        endcase
    end
end

always_ff @(posedge clk_i or negedge rstn_i) begin : memory_to_writebak_ff
    if(!rstn_i) begin
        pc_q_memory <= '0;
        instr_q_memory <= '0;
        rd_data_q_memory <= '0;
        rd_q_memory <= '0;
        rf_write_enable_q_memory <= '0;
    end else begin
        pc_q_memory <= pc_d_memory;
        instr_q_memory <= instr_d_memory;
        rd_data_q_memory <= rd_data_d_memory;
        rd_q_memory <= rd_d_memory;
        rf_write_enable_q_memory <= rf_write_enable_d_memory;
    end
end
//////////////////////////////////////////MEMORY AŞAMASI/////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////WRITE BACK AŞAMASI////////////////////////////////////////////////////////////////////////////////////////

    logic [XLEN-1:0] pc_d_writeback;
    assign pc_d_writeback = pc_q_memory;
    logic [XLEN-1:0] instr_d_writeback;
    assign instr_d_writeback = instr_q_memory;

    logic [XLEN-1:0] rd_data_d_writeback;
    assign rd_data_d_writeback = rd_data_q_memory;

    logic [     4:0] rd_d_writeback;
    assign rd_d_writeback = rd_q_memory;

    logic            rf_write_enable_d_writeback; 
    assign           rf_write_enable_d_writeback = rf_write_enable_q_memory;

    always_ff @(negedge clk_i or negedge rstn_i) begin //register file'a write back aşaması.
        if(!rstn_i) begin
            for(int i = 0; i<32; ++i) begin
                rf[i] <= '0; //eğer reset sinyali aktifse 32 adet register file'ın hepsine 0 değeri atıyoruz.
                end
            end  else if(rf_write_enable_d_writeback && rd_d_writeback != '0) begin //instr_d[11:7] değeri 0 olmadığı sürece olmasının sebebi RISC-V mimarisinde 0. register her zaman 0 değerini taşır.
                rf[rd_d_writeback] <= rd_data_d_writeback; //32 adet 32 bitlik register file'ımız olduğu için instr_d[11:7] yani 5 bitlik kısım yetiyor çünkü 2^5 = 32 adet register'a da ulaşmış oluyoruz. rd_data zaten 32 bit.
            end
    end
//////////////////////////////////////////WRITE BACK AŞAMASI//////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////HAZARD UNIT//////////////////////////////////////////////////////////////////////////////////////////////

    
    //FORWARD UNIT

    logic [1:0]      forward_rs1;
    logic [1:0]      forward_rs2;
    // Forward For RS1

    always_comb begin
        if(rf_write_enable_q_execute && (rd_q_execute != 0) && (rd_q_execute == rs1_addr_q_decode))
            forward_rs1 = 2'b01; // 01 ise execute aşamasından forward edilecek.
        else if(rf_write_enable_q_memory && (rd_q_memory != 0) && (rd_q_memory == rs1_addr_q_decode))
            forward_rs1 = 2'b10; // 10 ise memory aşamasından forward edilecek.
        else
            forward_rs1 = 2'b00; // 00 ise forward edilmeyecek.
    end

    // Forward For RS2
    
    always_comb begin
        if(rf_write_enable_q_execute && (rd_q_execute != 0) && (rd_q_execute == rs2_addr_q_decode))
            forward_rs2 = 2'b01; // 01 ise execute aşamasından forward edilecek.
        else if(rf_write_enable_q_memory && (rd_q_memory != 0) && (rd_q_memory == rs2_addr_q_decode))
            forward_rs2 = 2'b10; // 10 ise memory aşamasından forward edilecek.
        else
            forward_rs2 = 2'b00; // 00 ise forward edilmeyecek.
    end

    //FORWARD UNIT

endmodule
