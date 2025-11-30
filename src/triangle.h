/*****************************************************************************/
/*                                                                           */
/*  (triangle.h)                                                             */
/*                                                                           */
/*  为调用 Triangle 库的程序提供的头文件。                                  */
/*                                                                           */
/*  对应 Triangle 版本 1.6                                                  */
/*  2005 年 7 月 28 日                                                       */
/*                                                                           */
/*  Copyright 1996, 2005                                                     */
/*  Jonathan Richard Shewchuk                                                */
/*  2360 Woolsey #H                                                          */
/*  Berkeley, California  94705-1927                                         */
/*  jrs@cs.berkeley.edu                                                      */
/*                                                                           */
/*  Modified by Andreas Geiger, 2011                                         */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  如何在其他程序中调用 Triangle                                            */
/*                                                                           */
/*                                                                           */
/*  如果你还没有阅读 Triangle 的使用说明（可以在命令行运行 triangle -h）， */
/*  那么下面的内容可能比较难理解。                                          */
/*                                                                           */
/*  使用本头文件时，Triangle 需要以定义了 TRILIBRARY 宏的方式编译成对外     */
/*  可调用的目标文件（triangle.o），通常是在编译器参数中加入 -DTRILIBRARY   */
/*  选项。随 Triangle 附带的 makefile 中已经提供了相应规则，运行           */
/*  "make trilibrary" 即可生成该目标文件，然后通过 triangulate() 函数调 */
/*  用它。                                                                    */
/*                                                                           */
/*  如果你在意目标文件的大小，可以生成一个“精简版”的 triangle.o。定义      */
/*  REDUCED 宏会去掉主要用于科研目的的一些功能；更具体地说，使用           */
/*  -DREDUCED 选项会禁用 Triangle 的 -i、-F、-s 和 -C 等命令行开关。         */
/*  定义 CDT_ONLY 宏则会去掉除约束 Delaunay 三角剖分之外的其他网格生成算   */
/*  法；也就是说，-DCDT_ONLY 选项会禁用 Triangle 的 -r、-q、-a、-u、-D、-Y、 */
/*  -S 和 -s 等开关。                                                         */
/*                                                                           */
/*  重要说明：上述 TRILIBRARY、REDUCED、CDT_ONLY 这些宏**必须**在 makefile  */
/*  中或 triangle.c 源文件内部进行定义；仅仅在本头文件 triangle.h 中定义它们 */
/*  是不会产生预期效果的。                                                   */
/*                                                                           */
/*                                                                           */
/*  triangulate() 函数的调用约定如下所示。                                   */
/*                                                                           */
/*      void triangulate(triswitches, in, out, vorout)                       */
/*      char *triswitches;                                                   */
/*      struct triangulateio *in;                                            */
/*      struct triangulateio *out;                                           */
/*      struct triangulateio *vorout;                                        */
/*                                                                           */
/*  `triswitches' 是一个字符串，用来传入你希望启用的命令行开关；前面不需   */
/*  要加 `-' 号。下面是一些使用上的建议：                                    */
/*                                                                           */
/*  - 通常推荐使用 `z' 开关，使得点（以及其它对象）的编号从 0 开始。这会   */
/*    简化索引，因为无论编号是 0 还是 1，每种对象类型在对应数组中的第一个   */
/*    元素始终位于下标 [0] 位置。                                           */
/*  - 在最终发布的程序中，通常会使用 `Q'（安静模式）开关；但在调试阶段，     */
/*    你可以借助 Triangle 的文本输出（包括 `V' 开关）来查看更多信息。        */
/*  - 如果你没有使用 `q'、`a'、`u'、`D'、`j' 或 `s' 等开关，那么输出点集与   */
/*    输入点集将是完全相同的（边界标记可能除外）。如果不需要边界标记，可    */
/*    以加上 `N'（不输出节点）开关来节省内存。若既需要边界标记又想节省内存， */
/*    一个“小技巧”是：在调用 triangulate() 之前，将 out->pointlist 直接设  */
/*    为 in->pointlist，这样 Triangle 会在原数组上就地覆盖写入结果。         */
/*  - 当 Triangle 以 TRILIBRARY 方式编译时，`I'（不输出迭代次数）和 `g'       */
/*    （输出 .off 文件）这两个开关不会产生任何效果。                        */
/*                                                                           */
/*  参数 `in'、`out' 和 `vorout' 分别描述输入、输出以及 Voronoi 图输出。     */
/*  如果没有使用 `v'（Voronoi 输出）开关，则 `vorout' 可以为 NULL；但 `in'  */
/*  和 `out' 绝不能为 NULL。                                                */
/*                                                                           */
/*  部分输入和输出结构体中的字段在调用前必须被初始化，具体要求如下所述。    */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  `triangulateio' 结构体                                                   */
/*                                                                           */
/*  用于在 triangulate() 函数与外部程序之间传递数据。                        */
/*                                                                           */
/*                                                                           */
/*  本结构体中大量使用数组来保存点、三角形、标记等信息。对于任何数组，     */
/*  其第一个元素总是存放在下标 [0] 的位置。但除非使用 `z' 开关，否则该     */
/*  元素的“编号”是 1；只有在启用 `z' 开关时，这个元素的编号才是 0。因而， */
/*  如果你使用了 `z' 开关，就会更容易对点（以及邻接列表中的三角形等）进   */
/*  行索引——当然，如果你是从 Fortran 程序中调用 Triangle，则另当别论。      */
/*                                                                           */
/*  下面对各字段逐一说明（`numberof' 系列计数字段顾名思义，这里不再赘述）： */
/*                                                                           */
/*  `pointlist'：点坐标数组。第一个点的 x 坐标位于下标 [0]，y 坐标位于      */
/*    下标 [1]，后面依次存放其余点的坐标。每个点占用两个 REAL。              */
/*  `pointattributelist'：点属性数组。每个点的属性占用                         */
/*    `numberofpointattributes' 个 REAL。                                     */
/*  `pointmarkerlist'：点标记数组；每个点对应一个 int 标记。                   */
/*                                                                           */
/*  `trianglelist'：三角形顶点索引数组。第一个三角形的第一个顶点位于下标     */
/*    [0]，其余两个顶点按逆时针顺序紧随其后；若该三角形表示的是非线性单元， */
/*    则后面还会跟着额外的节点索引。每个三角形占用                            */
/*    `numberofcorners' 个 int。                                               */
/*  `triangleattributelist'：三角形属性数组。每个三角形的属性占用               */
/*    `numberoftriangleattributes' 个 REAL。                                   */
/*  `trianglearealist'：三角形面积约束数组；每个三角形对应一个 REAL。仅输入用。*/
/*  `neighborlist'：三角形邻接关系数组；每个三角形对应三个 int。仅输出用。    */
/*                                                                           */
/*  `segmentlist'：线段端点索引数组。第一条线段的两个端点位于下标 [0] 和 [1]， */
/*    后面依次是其余线段的信息。每条线段占用两个 int。                        */
/*  `segmentmarkerlist'：线段标记数组；每条线段对应一个 int 标记。             */
/*                                                                           */
/*  `holelist'：洞（空洞）列表。第一个洞的 x、y 坐标位于下标 [0] 和 [1]，   */
/*    后面依次是其余洞的坐标。每个洞占用两个 REAL。该字段仅用于输入，但指针 */
/*    会为了方便被拷贝到输出结构中。                                         */
/*                                                                           */
/*  `regionlist'：区域属性与面积约束列表。第一个约束的 x、y 坐标位于下标     */
/*    [0] 和 [1]，区域属性位于下标 [2]，最大面积约束位于下标 [3]，其后依次为  */
/*    其它区域的约束信息。每个区域约束占用四个 REAL。需要注意的是：只有在    */
/*    选择了 `A' 开关时才会使用区域属性；只有在选择 `a' 开关且后面不跟具体    */
/*    数字时才会使用面积约束；但即便没有选用这些开关，该数组的内存布局也不变。*/
/*    该字段仅用于输入，但指针会为了方便被拷贝到输出结构中。               */
/*                                                                           */
/*  `edgelist'：边的端点索引数组。第一条边的两个端点位于下标 [0] 和 [1]，    */
/*    后面依次是其余边的信息。每条边占用两个 int，仅输出用。                */
/*  `edgemarkerlist'：边标记数组；每条边对应一个 int 标记，仅输出用。         */
/*  `normlist'：用于 Voronoi 图中无穷远射线的法向量数组。第一个法向量的      */
/*    x、y 分量位于下标 [0] 和 [1]，后面依次是其余法向量。对于 Voronoi 图中   */
/*    的每一条有限边，其对应的法向量会被写成零向量。每条边占用两个 REAL，   */
/*    仅输出用。                                                              */
/*                                                                           */
/*                                                                           */
/*  Triangle 将会查看的所有输入字段都必须在调用前被正确初始化。此外，对于   */
/*  每一个 Triangle 要写入的输出数组，你要么事先分配好空间并把指针设置到    */
/*  正确的位置，要么将指针初始化为 NULL，表示让 Triangle 自行分配结果数组。  */
/*  建议使用后一种方式，因为 Triangle 最清楚需要分配多大的空间。前一种方    */
/*  式主要是为了从 Fortran 代码中调用 Triangle 的场景而提供，不过它也允许   */
/*  一些“省空间”的小技巧（例如将输出直接写入输入用的数组）。              */
/*                                                                           */
/*  Triangle 不会对任何输入或输出数组调用 free()，包括它自己分配的那些；    */
/*  释放内存的责任在于调用者。对于由 Triangle 分配的数组，建议调用下面定    */
/*  义的 trifree() 来释放。（trifree() 默认只是调用标准库的 free()；不过那   */
/*  些调用 triangulate() 的应用也可以在 triangle.c 中重写 trimalloc() 和     */
/*  trifree()，以便使用专门的内存分配器。）                                   */
/*                                                                           */
/*  下文给出了一份简要指南，帮助你在调用 triangulate() 之前判断哪些字段必    */
/*  须被初始化。                                                              */
/*                                                                           */
/*  `in'：                                                                   */
/*                                                                           */
/*    - `pointlist' 必须指向点坐标数组；`numberofpoints' 和                   */
/*      `numberofpointattributes' 必须设置正确。`pointmarkerlist' 要么设为   */
/*      NULL（此时所有标记默认值为 0），要么指向点标记数组。如果              */
/*      `numberofpointattributes' 不为 0，则 `pointattributelist' 必须指向点   */
/*      属性数组。                                                            */
/*    - 若使用 `r' 开关，则 `trianglelist' 必须指向三角形列表，并且           */
/*      `numberoftriangles'、`numberofcorners' 与 `numberoftriangleattributes' */
/*      都必须设置正确。若 `numberoftriangleattributes' 不为 0，则             */
/*      `triangleattributelist' 必须指向三角形属性数组。若使用 `a' 开关（且   */
/*      后面不跟数字），则 `trianglearealist' 必须指向面积约束数组。          */
/*      `neighborlist' 可忽略。                                               */
/*    - 若使用 `p' 开关，则 `segmentlist' 必须指向线段列表，`numberofsegments' */
/*      必须设置正确，并且 `segmentmarkerlist' 要么设为 NULL（所有标记默认为  */
/*      0），要么指向线段标记数组。                                           */
/*    - 若使用 `p' 开关但未使用 `r' 开关，则 `numberofholes' 与                */
/*      `numberofregions' 必须设置正确。若 `numberofholes' 不为 0，则          */
/*      `holelist' 必须指向洞列表；若 `numberofregions' 不为 0，则             */
/*      `regionlist' 必须指向区域约束列表。                                   */
/*    - 若使用 `p' 开关，则 `holelist'、`numberofholes'、`regionlist' 以及     */
/*      `numberofregions' 会被拷贝到 `out' 中。（如果同时使用了 `r' 开关，     */
/*      你也可以不对这些字段进行初始化。）                                   */
/*    - `edgelist'、`edgemarkerlist'、`normlist' 与 `numberofedges' 可以忽略。 */
/*                                                                           */
/*  `out'：                                                                  */
/*                                                                           */
/*    - 除非使用 `N' 开关，否则必须初始化 `pointlist'（设为 NULL 或指向已分配 */
/*      的内存）。除非同时使用 `N' 或 `B' 开关，否则必须初始化               */
/*      `pointmarkerlist'。若未使用 `N' 且 `in->numberofpointattributes' 不为  */
/*      0，则必须初始化 `pointattributelist'。                                */
/*    - 除非使用 `E' 开关，否则必须初始化 `trianglelist'。若使用 `n' 开关，   */
/*      则必须初始化 `neighborlist'。若未使用 `E' 且（`in->numberofelementattributes' */
/*      不为 0 或使用了 `A' 开关），则必须初始化 `elementattributelist'。      */
/*      `trianglearealist' 可以忽略。                                         */
/*    - 若使用 `p' 或 `c' 开关，且未使用 `P' 开关，则必须初始化                */
/*      `segmentlist'；在此情形下，除非同时使用 `B' 开关，否则也必须初始化    */
/*      `segmentmarkerlist'。                                                 */
/*    - 若使用 `e' 开关，则必须初始化 `edgelist'；如果使用了 `e' 但未使用      */
/*      `B' 开关，则还必须初始化 `edgemarkerlist'。                           */
/*    - `holelist'、`regionlist'、`normlist' 以及所有标量字段都可以忽略。      */
/*                                                                           */
/*  `vorout'（仅在使用 `v' 开关时需要）：                                     */
/*                                                                           */
/*    - 必须初始化 `pointlist'。若 `in->numberofpointattributes' 不为 0，      */
/*      则必须初始化 `pointattributelist'。`pointmarkerlist' 可以忽略。        */
/*    - 必须同时初始化 `edgelist' 和 `normlist'；`edgemarkerlist' 可以忽略。   */
/*    - 其它字段都可以忽略。                                                 */
/*                                                                           */
/*  调用 triangulate() 之后，`out' 和 `vorout' 中哪些字段是有效的，将以一种   */
/*  显而易见的方式取决于所使用的开关。需要注意的是，当使用 `p' 开关时，        */
/*  指针 `holelist' 和 `regionlist' 会从 `in' 拷贝到 `out'，但不会重新分配    */
/*  空间；因此要小心不要对同一数组调用两次 free()。另一方面，Triangle 永远 */
/*  不会简单地拷贝 `pointlist'（或其它类似指针）；它会为                      */
/*  `out->pointlist' 分配新的空间，或者在使用 `N' 开关时保持其未初始化状态。   */
/*                                                                           */
/*  所有有意义的 `numberof' 计数字段都会被正确设置；例如，无论是否实际写出  */
/*  边，`numberofedges' 都表示三角剖分中边的总数。若未使用线段约束，则        */
/*  `numberofsegments' 表示边界边的数量。                                    */
/*                                                                           */
/*****************************************************************************/

struct triangulateio {
  float *pointlist;                                               /* 输入 / 输出 */
  float *pointattributelist;                                      /* 输入 / 输出 */
  int *pointmarkerlist;                                          /* 输入 / 输出 */
  int numberofpoints;                                            /* 输入 / 输出 */
  int numberofpointattributes;                                   /* 输入 / 输出 */

  int *trianglelist;                                             /* 输入 / 输出 */
  float *triangleattributelist;                                   /* 输入 / 输出 */
  float *trianglearealist;                                         /* 仅输入 */
  int *neighborlist;                                             /* 仅输出 */
  int numberoftriangles;                                         /* 输入 / 输出 */
  int numberofcorners;                                           /* 输入 / 输出 */
  int numberoftriangleattributes;                                /* 输入 / 输出 */

  int *segmentlist;                                              /* 输入 / 输出 */
  int *segmentmarkerlist;                                        /* 输入 / 输出 */
  int numberofsegments;                                          /* 输入 / 输出 */

  float *holelist;                        /* 输入 / 指向将被拷贝到输出的数组 */
  int numberofholes;                                      /* 输入 / 计数会被拷贝 */

  float *regionlist;                      /* 输入 / 指向将被拷贝到输出的数组 */
  int numberofregions;                                    /* 输入 / 计数会被拷贝 */

  int *edgelist;                                                 /* 仅输出 */
  int *edgemarkerlist;            /* Voronoi 图中不用；仅输出 */
  float *normlist;                /* 仅在 Voronoi 图中使用；仅输出 */
  int numberofedges;                                             /* 仅输出 */
};

void triangulate(char *,triangulateio *,triangulateio *,triangulateio *);
void trifree(int *memptr);

