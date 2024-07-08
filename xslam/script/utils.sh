trap ctrl_c INT
function ctrl_c() {
    kill %1
    exit
}

function srcc() {
    prefix=''
    if [ "$#" -eq 1 ]; then
        prefix="_$1"
        print $prefix
    fi

    cwd=$(pwd)
    cdir=$(pwd)
    while [ $cdir != $HOME ]; do
        dir_to_check="$cdir/devel$prefix"
        if [ -d $dir_to_check ]; then
            source "$dir_to_check/setup.bash"
            echo "source $dir_to_check/setup.bash"
            break
        fi
        cd ..
        cdir=$(pwd)
    done
    if [ $cdir = $HOME ]; then
        echo "failed to find a catkin... "
    fi

    cd $cwd
}