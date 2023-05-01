cd build && make

if [ $? -ne 0 ]; then
    echo ""
    echo -e "\033[31m!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\033[0m"
    echo -e "\033[31mCATKIN_MAKE FAILED\033[0m"
    echo -e "\033[31m!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\033[0m"
    exit
fi

./potential_passing