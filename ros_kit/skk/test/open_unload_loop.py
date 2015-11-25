from primesense import nite2
from primesense import openni2

nite2.initialize()
ut=nite2.UserTracker.open_any()
print nite2.is_initialized()

nite2.unload()
print nite2.is_initialized()

nite2.initialize()
print nite2.is_initialized()
ut2=nite2.UserTracker.open_any()

nite2.unload()
