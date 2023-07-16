package script;

import java.util.*;
import java.io.*;

   // from: http://www.javaworld.com/javaworld/jw-12-2000/jw-1229-traps.html
   public class NativeExecute2
   {
       int exitVal;

       // from: http://www.javaworld.com/javaworld/jw-12-2000/jw-1229-traps.html
       class StreamGobbler extends Thread
       {
	   InputStream is;
	   String type;

	   StreamGobbler( InputStream is, String type )
	   {
	       this.is = is;
	       this.type = type;
	   }

	   public void run( )
	   {
	       try
		   {
		       InputStreamReader isr = new InputStreamReader( is );
		       BufferedReader br = new BufferedReader( isr );
		       String line=null;
		       while ( (line = br.readLine( )) != null)
			   System.out.println( type + ">" + line );
		   } catch ( IOException ioe )
		   {
		       ioe.printStackTrace( );
		   }
	   }
       }

       public int getExitValue() {
	   return exitVal;
       }

       
       public NativeExecute2( String[] cmd/*, File workingDirFile*/ )
       {
           try
           {
//                System.out.println( "=======================" );
//                System.out.println( "EXEC: " + cmd );
//                System.out.println( "CURRENT WORKING DIR: " + System.getProperty( "user.dir" ) );
// //                System.out.println( "NEW WORKING DIR: " + workingDirFile.getPath(
// // ) );
//                System.out.println( "=======================" );

               Runtime rt = Runtime.getRuntime( );

               // System.setProperty( "user.dir", workingDirFile.getPath() );
               // Process proc = rt.exec( cmd, null, workingDirFile );
               Process proc = rt.exec( cmd );

               // Any error message?
               StreamGobbler errorGobbler = new
                   StreamGobbler( proc.getErrorStream( ), "ERROR" );

               // any output?
               StreamGobbler outputGobbler = new
                   StreamGobbler( proc.getInputStream( ), "OUTPUT" );

               // kick them off
               errorGobbler.start( );
               outputGobbler.start( );

               // any error???
               exitVal = proc.waitFor( );
               System.out.println( "Exit value: " + exitVal );
           } 
           catch ( Throwable t )
           {
               t.printStackTrace( );
           }
       }
   }
