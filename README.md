My Balcony Project
----

Getting lots of sun, and very little precipitation, two days without watering,
and my plants on the balcony are dead. Long weekends, never mind holidays,
disastrous. So lets water them automatically.

There is a large bucket of water on the floor.

Hanging at about 2 meters is a plastic balcony plant holder. I selected it for
having only one drain hole. Replaced the hole with a tube outlet. From there
there is tubing to my plants. Plants are at 1 meter height or on the floor.
Once water is in this "bucket" gravity takes care of the rest.

There is a water pump in the bottom bucket, and a hose running to the high
bucket.

I have a 12V 7Amp battery, hooked up to a solar panel. There is a water pump
connected to it via a relays. There is an arduino that can switch the relays.

The arduino is connected to:
1. a relays
2. a moist sensor (glued in the high bucket)
3. a (emergency) stop button

For now, the arduino drives the pump, until it senses water. It also stops when the button is pressed, or when 1 minute passed. It doesn't take long to fill the top bucket, and the pump may not be run continuously, or run dry.

TODO
----

larger bottom bucket
sensor in the bottom bucket
add time to arduino, pump once every 24 hours
replace sensor in top bucket (maybe)
alert when bottom bucket is empty
allow setting amount of water to be pumped, perhaps temperature based etc.

