# (1)コンパイラ
CC  = g++
# (2)コンパイルオプション
CFLAGS    = -std=c++1z 
# (3)実行ファイル名
TARGET  = pre-mbdyn
# (4)コンパイル対象のソースコード
SRCS     = Vec3d.cc
SRCS    += Frame.cc
SRCS    += ReferenceFrame.cc
SRCS    += Node.cc
SRCS    += Element.cc
SRCS    += RigidBody.cc
SRCS    += Joint.cc
SRCS    += Platform.cc
SRCS    += Tower.cc
SRCS    += Nacelle.cc
SRCS    += Blade.cc
SRCS    += main.cc
SRCS    += InputData.cc
# (5)オブジェクトファイル名
OBJS    = $(SRCS:.cpp=.o)
 
# (6)インクルードファイルのあるディレクトリパス
INCDIR  = -I../inc
 
# (7)ライブラリファイルのあるディレクトリパス
LIBDIR  = 
 
# (8)追加するライブラリファイル
LIBS    = 

# (9)ターゲットファイル生成
$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LIBDIR) $(LIBS)
	
# (10)オブジェクトファイル生成
$(OBJS): $(SRCS)
	$(CC) $(CFLAGS) $(INCDIR) -c $(SRCS)

# (11)"make all"で make cleanとmakeを同時に実施。
all: clean $(OBJS) $(TARGET)
# (12).oファイル、実行ファイル、.dファイルを削除
clean:
	-rm -f $(OBJS) $(TARGET) *.d

install:
	mkdir -p $(DESTDIR)/usr/bin
	install -m 0755 pre-mbdyn $(DESTDIR)/usr/bin/pre-mbdyn